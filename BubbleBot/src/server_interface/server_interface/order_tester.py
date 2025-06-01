#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from base_interfaces.srv import BaseDBCommand, OrderRequest


class TestAllServices(Node):
    def __init__(self):
        super().__init__('test_all_services')
        self.cli_db = self.create_client(BaseDBCommand, 'db_command')
        self.cli_order = self.create_client(OrderRequest, 'order_request')

    def wait_services(self):
        self.get_logger().info("⏳ Waiting for services...")
        self.cli_db.wait_for_service()
        self.cli_order.wait_for_service()

    def call_db(self, **kwargs):
        req = BaseDBCommand.Request()
        for k, v in kwargs.items():
            setattr(req, k, v)
        future = self.cli_db.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_order(self, recipe_name, sugar, ice, toppings):
        req = OrderRequest.Request()
        req.recipe_name = recipe_name
        req.sugar_level = sugar
        req.ice_level = ice
        req.toppings = toppings
        future = self.cli_order.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_tests(self):
        self.wait_services()

        # def add_station_and_get_id(name, x, y, z):
        #     result = self.call_db(
        #         command='add_station',
        #         name=name,
        #         station_type='material',
        #         description='', liquid_ingredients='', default_toppings='', branch_name='',
        #         type=0, station_id=0, is_liquid=0, is_topping=0, topping_position='',
        #         loc_x=x, loc_y=y, loc_z=z,
        #         loc_ox=0.0, loc_oy=0.0, loc_oz=0.0, loc_ow=1.0
        #     )
        #     self.get_logger().info(result.message)

        #     # extract station id from response.message
        #     if "id=" in result.message:
        #         return int(result.message.split("id=")[-1])
        #     else:
        #         # fallback: re-query station id
        #         confirm = self.call_db(
        #             command='get_materials',
        #             name='', description='', liquid_ingredients='', default_toppings='',
        #             type=0, station_id=0, is_liquid=0, is_topping=0, topping_position='',
        #             branch_name='', loc_x=0.0, loc_y=0.0, loc_z=0.0, loc_ox=0.0, loc_oy=0.0, loc_oz=0.0, loc_ow=1.0
        #         )
        #         return 1  # just fallback

        # def add_material(name, type_, is_liquid, is_topping, topping_pos, station_id, x, y, z):
        #     result = self.call_db(
        #         command='add_material',
        #         name=name,
        #         type=type_,
        #         station_id=station_id,
        #         is_liquid=is_liquid,
        #         is_topping=is_topping,
        #         topping_position=topping_pos,
        #         description='',
        #         liquid_ingredients='',
        #         default_toppings='',
        #         branch_name='',
        #         loc_x=x, loc_y=y, loc_z=z,
        #         loc_ox=0.0, loc_oy=0.0, loc_oz=0.0, loc_ow=1.0
        #     )
        #     self.get_logger().info(result.message)

        # # --- Test Sequence ---

        # # Add stations
        # tea_station_id = add_station_and_get_id("leo_station", 1.2, -0.2, 0.0)
        # coco_station_id = add_station_and_get_id("jason_station", 1.7, 0.3, 0.0)

        # # Add materials
        # add_material("leo", 1, 1, 0, 'none', tea_station_id, 1.2, -0.2, 0.0)
        # add_material("jason", 2, 0, 1, 'top', coco_station_id, 1.7, 0.3, 0.0)

        # # Add recipe
        # result = self.call_db(
        #     command='add_recipe',
        #     name='Milk Tea',
        #     description='A classic drink',
        #     liquid_ingredients='tea,milk',
        #     default_toppings='coco',
        #     type=0, station_id=0, is_liquid=0, is_topping=0, topping_position='',
        #     branch_name='', loc_x=0.0, loc_y=0.0, loc_z=0.0, loc_ox=0.0, loc_oy=0.0, loc_oz=0.0, loc_ow=1.0
        # )
        # self.get_logger().info(result.message)

        # Query
        # result = self.call_db(command='get_materials')
        # self.get_logger().info(f"Materials: {result.material_names}")
        # result = self.call_db(command='get_recipes')
        # self.get_logger().info(f"Recipes: {result.recipe_names}")

        # Orders
        for i in range(3):
            result = self.call_order('Pearl Milk Tea', '50%', 'less ice', 'coco')
            self.get_logger().info(result.message)
            time.sleep(1)

        # Wait for dispatch to complete
        time.sleep(10)

        # Delete
        # result = self.call_db(command='delete_material', name='coco')
        # self.get_logger().info(result.message)
        # result = self.call_db(command='delete_recipe', name='Pearl Milk Tea')
        # self.get_logger().info(result.message)

            # 🔁 NEW: Switch DB branch to 'fruit'
        # result = self.call_db(command='switch_db', branch_name='fruit')
        # self.get_logger().info(result.message)


def main(args=None):
    rclpy.init(args=args)
    node = TestAllServices()
    node.run_tests()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()