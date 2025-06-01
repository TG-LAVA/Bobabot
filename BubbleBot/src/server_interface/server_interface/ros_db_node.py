#!/usr/bin/env python3
import os
import sqlite3
import threading
import queue


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from base_interfaces.srv import BaseDBCommand, OrderRequest, WorkFlow, BaseCommand
from geometry_msgs.msg import PoseStamped, PointStamped


class RecipeDatabaseNode(Node):
    def __init__(self):
        super().__init__('recipe_database_node')

        # --- paths ---
        pkg_share = get_package_share_directory('server_interface')
        self.schema_dir = os.path.join(pkg_share, 'sql')
        self.db_path = os.path.join(self.schema_dir, 'bubbletea.db')

        self.load_default_recipe('classic')
        # --- services ---
        self.create_service(BaseDBCommand, 'db_command', self.handle_db_command)
        self.create_service(OrderRequest,  'order_request', self.handle_order_request)
        self.create_service(BaseCommand,  'db_base_command', self.handle_db_command)

        # client for dispatching the workflow
        self.workflow_client = self.create_client(WorkFlow, 'workflow')

        self.dispatch_queue = queue.Queue()
        self.dispatch_lock = threading.Lock()
        self.dispatching = set()

        self.get_logger().info("🍵 RecipeDatabaseNode ready")

    def connect(self):
        return sqlite3.connect(self.db_path)

    #  --- BaseDBCommand handler: add/delete/list materials & recipes, load_branch ---
    def handle_db_command(self, request, response):
        cmd = request.command.lower()
        conn = self.connect()
        cur = conn.cursor()
        try:
            if cmd == 'add_station':
                # 插入或更新 station（按 name 匹配）
                cur.execute(
                    """
                    INSERT INTO stations (name, type, loc_x, loc_y, loc_z, loc_ox, loc_oy, loc_oz, loc_ow)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ON CONFLICT(name) DO UPDATE SET
                        type    = excluded.type,
                        loc_x   = excluded.loc_x,
                        loc_y   = excluded.loc_y,
                        loc_z   = excluded.loc_z,
                        loc_ox  = excluded.loc_ox,
                        loc_oy  = excluded.loc_oy,
                        loc_oz  = excluded.loc_oz,
                        loc_ow  = excluded.loc_ow
                    """,
                    (
                        request.name,                     # station name
                        request.station_type,             # 'material' or 'operation'
                        request.loc_x,
                        request.loc_y,
                        request.loc_z,
                        request.loc_ox,
                        request.loc_oy,
                        request.loc_oz,
                        request.loc_ow
                    )
                )
                # 获取 station_id（无论插入还是更新）
                cur.execute("SELECT id FROM stations WHERE name=?", (request.name,))
                row = cur.fetchone()
                conn.commit()

                if row:
                    response.success = True
                    response.message = f"✅ Station inserted or updated with id={row[0]}"
                else:
                    response.success = False
                    response.message = "❌ Station insertion/update failed"

            elif cmd == 'add_material':
                # 增加或更新 material，同时写入自己的位姿
                cur.execute("SELECT id FROM stations WHERE id=?", (request.station_id,))
                if cur.fetchone() is None:
                    raise RuntimeError(f"Station id {request.station_id} not found")

                cur.execute(
                    """
                    INSERT INTO materials (
                        name, type, enabled, station_id, is_liquid, is_topping, topping_position,
                        loc_x, loc_y, loc_z, loc_ox, loc_oy, loc_oz, loc_ow
                    )
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ON CONFLICT(name) DO UPDATE SET
                        type             = excluded.type,
                        enabled          = excluded.enabled,
                        station_id       = excluded.station_id,
                        is_liquid        = excluded.is_liquid,
                        is_topping       = excluded.is_topping,
                        topping_position = excluded.topping_position,
                        loc_x            = excluded.loc_x,
                        loc_y            = excluded.loc_y,
                        loc_z            = excluded.loc_z,
                        loc_ox           = excluded.loc_ox,
                        loc_oy           = excluded.loc_oy,
                        loc_oz           = excluded.loc_oz,
                        loc_ow           = excluded.loc_ow
                    """,
                    (
                        request.name,
                        request.type,
                        1,
                        request.station_id,
                        request.is_liquid,
                        request.is_topping,
                        None if not request.is_topping or request.topping_position.lower() == 'none'
                            else request.topping_position,
                        request.loc_x,
                        request.loc_y,
                        request.loc_z,
                        request.loc_ox,
                        request.loc_oy,
                        request.loc_oz,
                        request.loc_ow
                    )
                )
                conn.commit()
                response.success = True
                response.message = "✅ Material inserted or updated"

            elif cmd == 'delete_material':
                # 删除 material
                cur.execute("DELETE FROM materials WHERE name=?", (request.name,))
                conn.commit()
                response.success = True
                response.message = "✅ Material deleted"

            elif cmd == 'delete_station':
                # 删除 station（必须先无材料引用）
                cur.execute("SELECT COUNT(*) FROM materials WHERE station_id=?", (request.station_id,))
                if cur.fetchone()[0] > 0:
                    raise RuntimeError("Cannot delete station: still referenced by materials")
                cur.execute("DELETE FROM stations WHERE id=?", (request.station_id,))
                conn.commit()
                response.success = True
                response.message = "✅ Station deleted"


            elif cmd == 'add_recipe':
                cur.execute(
                    "INSERT INTO recipes (name,description,liquid_ingredients,default_toppings) "
                    "VALUES (?,?,?,?)",
                    (
                        request.name,
                        request.description,
                        request.liquid_ingredients,
                        request.default_toppings
                    )
                )
                conn.commit()
                response.success = True
                response.message = "✅ Recipe added"

            elif cmd == 'delete_recipe':
                cur.execute("DELETE FROM recipes WHERE name=?", (request.name,))
                conn.commit()
                response.success = True
                response.message = "✅ Recipe deleted"

            elif cmd == 'get_materials':
                cur.execute("SELECT name,type,topping_position FROM materials")
                rows = cur.fetchall()
                response.material_names = [r[0] for r in rows]
                response.types          = [str(r[1]) for r in rows]
                response.positions      = [r[2] or '' for r in rows]
                response.success = True
                response.message = f"✅ {len(rows)} materials"

            elif cmd == 'get_recipes':
                cur.execute(
                    "SELECT name,description,liquid_ingredients,default_toppings FROM recipes"
                )
                rows = cur.fetchall()
                response.recipe_names = [r[0] for r in rows]
                response.descriptions = [r[1] for r in rows]
                response.liquids      = [r[2] for r in rows]
                response.toppings     = [r[3] for r in rows]
                response.success = True
                response.message = f"✅ {len(rows)} recipes"

            elif cmd == 'get_home':
                self.get_logger().info("🔍 Received 'get_home' command, looking for station named 'home_station'")
                cur.execute("SELECT loc_x, loc_y, loc_z, loc_ox, loc_oy, loc_oz, loc_ow FROM stations WHERE name='home_station'")
                row = cur.fetchone()
                if row:
                    self.get_logger().info(f"✅ Found home_station at: x={row[0]}, y={row[1]}, z={row[2]}")
                    response.success = True
                    response.message = "Home station found"
                    response.loc_x, response.loc_y, response.loc_z, response.loc_ox, response.loc_oy, response.loc_oz, response.loc_ow = row
                else:
                    self.get_logger().warn("❌ home_station not found in database")
                    response.success = False
                    response.message = "No home_station defined"


            elif cmd == 'load_branch':
                fn = os.path.join(self.schema_dir, f"recipes_{request.branch_name}.sql")
                if not os.path.exists(fn):
                    response.success = False
                    response.message = f"❌ Branch file not found: {fn}"
                else:
                    with open(fn, 'r') as f:
                        cur.executescript(f.read())
                    conn.commit()
                    response.success = True
                    response.message = f"✅ Loaded branch '{request.branch_name}'"


            elif cmd == 'switch_db':
                branch = request.branch_name.strip()
                if not branch:
                    response.success = False
                    response.message = "❌ No branch name provided"
                else:
                    # 构造新 DB 路径
                    db_filename = f"bubbletea_{branch}.db"
                    new_db = os.path.join(self.schema_dir, db_filename)

                    # 如果不存在，就用模板 SQL 创建
                    if not os.path.isfile(new_db):
                        template = os.path.join(self.schema_dir, 'recipe_template.sql')
                        if not os.path.isfile(template):
                            response.success = False
                            response.message = f"❌ Template not found: {template}"
                            return response

                        try:
                            # 用 sqlite3 创建并执行脚本
                            conn2 = sqlite3.connect(new_db)
                            cur2 = conn2.cursor()
                            with open(template, 'r') as f:
                                cur2.executescript(f.read())
                            conn2.commit()
                            conn2.close()
                            self.get_logger().info(f"📦 Created new DB from template: {new_db}")
                        except Exception as e:
                            response.success = False
                            response.message = f"❌ Failed to create DB: {e}"
                            return response

                    # 切换到新 DB
                    self.db_path = new_db
                    response.success = True
                    response.message = f"✅ Switched to DB branch '{branch}' at {new_db}"

            else:
                response.success = False
                response.message = f"❌ Unknown command '{cmd}'"

        except Exception as e:
            response.success = False
            response.message = f"❌ {e}"
        finally:
            conn.close()
        return response
    
    def load_default_recipe(self, branch_name: str):
        fn = os.path.join(self.schema_dir, f"recipes_{branch_name}.sql")
        if not os.path.isfile(fn):
            self.get_logger().warn(f"Default recipe file not found: {fn}")
            return

        conn = self.connect()
        cur = conn.cursor()
        try:
            with open(fn, 'r') as f:
                cur.executescript(f.read())
            conn.commit()
            self.get_logger().info(f"✅ Loaded default recipes branch '{branch_name}'")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load default recipes: {e}")
        finally:
            conn.close()


    #  --- OrderRequest handler: insert order, gen workflow, then dispatch asynchronously ---
    def handle_order_request(self, request, response):
        conn = self.connect()
        cur = conn.cursor()
        try:
            # find recipe_id
            cur.execute("SELECT id FROM recipes WHERE name=?", (request.recipe_name,))
            row = cur.fetchone()
            if not row:
                response.success = False
                response.message = "❌ Recipe not found"
                return response
            recipe_id = row[0]

            # insert order
            cur.execute(
                "INSERT INTO orders (recipe_id,sugar_level,ice_level,toppings,status) VALUES (?,?,?,?,?)",
                (recipe_id, request.sugar_level, request.ice_level, request.toppings, 'unprepared')
            )
            order_id = cur.lastrowid
            conn.commit()

            # build workflow_steps
            self.generate_workflow_from_order(order_id)

            response.success = True
            response.message = f"✅ Order #{order_id} saved"

            # dispatch in background（只有队列原本空才启动一次）
            with self.dispatch_lock:
                was_empty = self.dispatch_queue.empty()
                if order_id not in list(self.dispatch_queue.queue):
                    self.dispatch_queue.put(order_id)
                if was_empty:
                    threading.Thread(target=self._try_dispatch_from_queue, daemon=True).start()

        except Exception as e:
            response.success = False
            response.message = str(e)
        finally:
            conn.close()
        return response

    def _update_order_status(self, order_id: int, status: str):
        conn = self.connect()
        cur = conn.cursor()
        cur.execute("UPDATE orders SET status=? WHERE id=?", (status, order_id))
        conn.commit()
        conn.close()

    def _get_station_id(self, cur, station_name: str) -> int:
        cur.execute("SELECT id FROM stations WHERE name=?", (station_name,))
        return cur.fetchone()[0]

    def generate_workflow_from_order(self, order_id: int):
        conn = self.connect()
        cur = conn.cursor()
        try:
            # 获取订单配方数据
            cur.execute("""
                SELECT o.recipe_id, o.toppings, r.liquid_ingredients
                FROM orders o JOIN recipes r ON o.recipe_id = r.id WHERE o.id=?
            """, (order_id,))
            recipe_id, tops_str, liqs_str = cur.fetchone()
            toppings = tops_str.split(',') if tops_str else []
            liquids = liqs_str.split(',') if liqs_str else []

            steps = []
            idx = 1
            pouring_sid = self._get_station_id(cur, 'pouring_spot')
            discard_sid = self._get_station_id(cur, 'discard_station')
            cup_sid = self._get_station_id(cur, 'cup_station')

            # 分离 bottom/top toppings
            bottom_toppings = []
            top_toppings = []
            for t in toppings:
                cur.execute("""
                    SELECT topping_position FROM materials 
                    WHERE name=? AND is_topping=1
                """, (t,))
                row = cur.fetchone()
                if row:
                    if row[0] == 'bottom':
                        bottom_toppings.append(t)
                    elif row[0] == 'top':
                        top_toppings.append(t)

            # 处理 bottom toppings
            for i, t in enumerate(bottom_toppings):
                cur.execute("""
                    SELECT station_id FROM materials 
                    WHERE name=? AND is_topping=1 AND topping_position='bottom'
                """, (t,))
                r = cur.fetchone()
                if r:
                    sid = r[0]
                    steps += [
                        (order_id, recipe_id, idx,   'navigate',      t, sid),
                        (order_id, recipe_id, idx+1, 'pick_topping',  t, sid),
                        (order_id, recipe_id, idx+2, 'navigate',      None, pouring_sid),
                        (order_id, recipe_id, idx+3, 'pour_into_cup', t, sid),
                    ]
                    idx += 4

                    # 如果还有下一个 bottom topping 要操作，则立即丢杯子并取新杯
                    if i < len(bottom_toppings) - 1:
                        steps += [
                            (order_id, recipe_id, idx,   'navigate',          None, discard_sid),
                            (order_id, recipe_id, idx+1, 'discard_small_cup', 'rubbish_bin', discard_sid),
                            (order_id, recipe_id, idx+2, 'navigate',          None, cup_sid),
                            (order_id, recipe_id, idx+3, 'get_small_cup',     'small_cup', cup_sid),
                        ]
                        idx += 4

            # 如果没有 bottom toppings 且需要 liquids，先取杯子
            if not bottom_toppings and liquids:
                steps += [
                    (order_id, recipe_id, idx,   'navigate',        None, cup_sid),
                    (order_id, recipe_id, idx+1, 'get_small_cup',   'small_cup', cup_sid),
                ]
                idx += 2

            # 处理 liquids
            for i, l in enumerate(liquids):
                cur.execute("""
                    SELECT station_id FROM materials WHERE name=? AND is_liquid=1
                """, (l,))
                r = cur.fetchone()
                if r:
                    sid = r[0]
                    steps += [
                        (order_id, recipe_id, idx,   'navigate',      l, sid),
                        (order_id, recipe_id, idx+1, 'pick_liquid',   l, sid),
                        (order_id, recipe_id, idx+2, 'navigate',      None, pouring_sid),
                        (order_id, recipe_id, idx+3, 'pour_into_cup', l, sid),
                    ]
                    idx += 4

                    if i == len(liquids) - 1:
                        steps += [
                            (order_id, recipe_id, idx,   'navigate',          None, discard_sid),
                            (order_id, recipe_id, idx+1, 'discard_small_cup', 'rubbish_bin', discard_sid),
                        ]
                        idx += 2

            # 处理 top toppings
            for t in top_toppings:
                cur.execute("""
                    SELECT station_id FROM materials 
                    WHERE name=? AND is_topping=1 AND topping_position='top'
                """, (t,))
                r = cur.fetchone()
                if r:
                    sid = r[0]
                    steps += [
                        (order_id, recipe_id, idx,   'navigate',         t, sid),
                        (order_id, recipe_id, idx+1, 'pick_topping',     t, sid),
                        (order_id, recipe_id, idx+2, 'navigate',         None, pouring_sid),
                        (order_id, recipe_id, idx+3, 'sprinkle_on_top',  t, sid),
                    ]
                    idx += 4

            # 最后保险丢一次杯子
            steps += [
                (order_id, recipe_id, idx,   'navigate',          None, discard_sid),
                (order_id, recipe_id, idx+1, 'discard_small_cup', 'rubbish_bin', discard_sid),
            ]
            idx += 2

            cur.executemany("""
                INSERT INTO workflow_steps 
                (order_id, recipe_id, step_order, action, material, station_id)
                VALUES (?,?,?,?,?,?)
            """, steps)
            conn.commit()
            self.get_logger().info(f"✅ Workflow steps for order #{order_id} created")

        except Exception as e:
            self.get_logger().error(f"❌ Error gen workflow: {e}")
        finally:
            conn.close()

    def _try_dispatch_from_queue(self):
        with self.dispatch_lock:
            if self.dispatch_queue.empty():
                return
            order_id = self.dispatch_queue.queue[0]  # peek
            if order_id in self.dispatching:
                return  # 正在派发中

            self.dispatching.add(order_id)

        # 调用实际派发逻辑
        self._dispatch_order(order_id)

    def _dispatch_order(self, order_id: int):
        if not self.workflow_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ workflow unavailable")
            self._update_order_status(order_id, 'failed')
            return

        conn = self.connect()
        cur = conn.cursor()
        cur.execute(
            "SELECT action, material, station_id FROM workflow_steps WHERE order_id=? ORDER BY step_order",
            (order_id,)
        )
        steps = cur.fetchall()

        rows = []
        for act, material, station_id in steps:
            if act == 'navigate':
                cur.execute(
                    "SELECT loc_x, loc_y, loc_z, loc_ox, loc_oy, loc_oz, loc_ow FROM stations WHERE id=?",
                    (station_id,)
                )
            else:
                cur.execute(
                    "SELECT loc_x, loc_y, loc_z, loc_ox, loc_oy, loc_oz, loc_ow FROM materials WHERE name=?",
                    (material,)
                )
            r = cur.fetchone()
            if r:
                rows.append((act, *r))
            else:
                self.get_logger().warn(f"⚠️ Missing pose for {act}: material='{material}', station_id={station_id}")

        conn.close()

        if not rows:
            self.get_logger().warn(f"⚠️ No workflow for order #{order_id}")
            self._update_order_status(order_id, 'failed')
            return

        req = WorkFlow.Request()
        grab_map = {
            'pick_topping':1, 'pick_liquid':4,
            'pour_into_cup':3, 'sprinkle_on_top':3,
            'discard_small_cup':2, 'get_small_cup': 1,
        }

        for act, x, y, z, ox, oy, oz, ow in rows:
            try:
                xf, yf, zf = float(x), float(y), float(z)
                qx, qy, qz, qw = float(ox), float(oy), float(oz), float(ow)
            except:
                self.get_logger().warn(f"Skipping bad coords in '{act}'")
                continue

            req.grabberstatus.append(grab_map.get(act, 0))

            ps = PoseStamped()
            ps.header.frame_id = 'base_link'
            ps.pose.position.x = xf
            ps.pose.position.y = yf
            ps.pose.position.z = zf
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            req.turtlebotgoal.append(ps)

            pt = PointStamped()
            pt.header.frame_id = 'base_link'
            pt.point.x = xf
            pt.point.y = yf
            pt.point.z = zf
            req.armgoal.append(pt)

        # 异步调用 + 回调
        future = self.workflow_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_dispatch_response(fut, order_id))

    def _on_dispatch_response(self, future, order_id: int):
        try:
            res = future.result()
            success = res.success
        except Exception as e:
            self.get_logger().error(f"❌ Dispatch exception: {e}")
            success = False

        # 更新数据库状态
        new_status = 'preparing' if success else 'failed'
        self._update_order_status(order_id, new_status)

        # 如果成功才弹出队首
        with self.dispatch_lock:
            if success and not self.dispatch_queue.empty() and self.dispatch_queue.queue[0] == order_id:
                self.dispatch_queue.get()
            self.dispatching.discard(order_id)

        if success:
            self.get_logger().info(f"✅ Dispatch succeeded for order #{order_id}")
            # 成功后立即处理下一个
            threading.Thread(target=self._try_dispatch_from_queue, daemon=True).start()
        else:
            self.get_logger().warn(f"❌ Dispatch failed for order #{order_id}, will retry in 10s")
            # 10s 后重试同一订单
            threading.Timer(10.0, self._try_dispatch_from_queue).start()


def main(args=None):
    rclpy.init(args=args)
    node = RecipeDatabaseNode()

    # 用 MultiThreadedExecutor 而不是默认的单线程 executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()       # 会并行处理服务回调和 spin_until_future_complete
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
