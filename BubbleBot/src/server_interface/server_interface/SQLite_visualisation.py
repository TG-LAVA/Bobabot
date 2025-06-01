import sqlite3
import pydot
import os

DB_PATH = "/home/huiyu/turtlebot_ws/install/server_interface/share/server_interface/sql/bubbletea.db"
OUTPUT_DIR = "./outputs"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def draw_workflow_flowchart(order_id=1, mode="horizontal"):
    """
    Draws a workflow chart for the given order_id.
    Modes:
        - "horizontal": Left to right
        - "vertical": Top to bottom
        - "doubleline": Zigzag 2-line layout
    """
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        SELECT step_order, action, material, station_id 
        FROM workflow_steps 
        WHERE order_id=? ORDER BY step_order
    """, (order_id,))
    rows = cursor.fetchall()
    conn.close()

    if not rows:
        print(f"Order {order_id} has no workflow_steps.")
        return

    if mode == "vertical":
        graph = pydot.Dot(graph_type='digraph', rankdir="TB")
    else:
        graph = pydot.Dot(graph_type='digraph', rankdir="LR")

    if mode != "doubleline":
        last_node = None
        for i, (step_order, action, material, station_id) in enumerate(rows):
            label = f"{step_order+1}. {action}"
            if material:
                label += f"\n{material}"
            if station_id:
                label += f"\n(station {station_id})"
            node = pydot.Node(f"step_{i}", label=label, shape="box", style="filled", fillcolor="#AED6F1")
            graph.add_node(node)
            if last_node:
                graph.add_edge(pydot.Edge(last_node, node))
            last_node = node

        # Add start circle
        start = pydot.Node("start", label="Start", shape="circle", style="filled", fillcolor="lightgray")
        graph.add_node(start)
        graph.add_edge(pydot.Edge(start, "step_0"))

    else:
        # Two-row zigzag layout
        nodes = []
        for i, (step_order, action, material, station_id) in enumerate(rows):
            label = f"{step_order+1}. {action}"
            if material:
                label += f"\n{material}"
            if station_id:
                label += f"\n(station {station_id})"
            node = pydot.Node(
                f"step_{i}", 
                label=label, 
                shape="box", 
                style="filled", 
                fillcolor="#AED6F1"
            )
            nodes.append(node)
            graph.add_node(node)

        # Add start node pointing to the first step
        start = pydot.Node("start", label="Start", shape="circle", style="filled", fillcolor="lightgray")
        graph.add_node(start)
        graph.add_edge(pydot.Edge(start, nodes[0]))

        # Zigzag connections
        steps_per_row = 4
        for i in range(len(nodes) - 1):
            current_row = i // steps_per_row
            if current_row % 2 == 0:
                graph.add_edge(pydot.Edge(nodes[i], nodes[i+1]))
            else:
                graph.add_edge(pydot.Edge(nodes[i+1], nodes[i]))

    suffix = mode
    path = os.path.join(OUTPUT_DIR, f"workflow_order_{suffix}.png")
    graph.write_png(path)
    print(f"Workflow diagram saved to {path}")

# Example usage
if __name__ == "__main__":
    draw_workflow_flowchart(order_id=1, mode="horizontal")
    draw_workflow_flowchart(order_id=1, mode="vertical")
    draw_workflow_flowchart(order_id=1, mode="doubleline")
