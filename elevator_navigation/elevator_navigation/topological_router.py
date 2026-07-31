import networkx as nx
import math

class TopologicalGraphRouter:
    def __init__(self, yaml_config):
        self.config = yaml_config
        self.graph = nx.DiGraph()
        self.elevators_by_floor = {}
        self._build_graph()

    def _build_graph(self):
        """Constructs the permanent node graph using integer floor IDs."""
        for bldg_name, b_data in self.config['buildings'].items():
            for floor_id, f_data in b_data['floors'].items():
                floor_id = int(floor_id)  # Ensure floor_id is an integer (-1, 0, 1, 2, etc.)
                floor_key = f"{bldg_name}_{floor_id}"
                self.elevators_by_floor[floor_key] = []
                
                for elev_name, e_data in f_data.get('elevators', {}).items():
                    node_id = f"{bldg_name}_{floor_id}_{elev_name}"
                    self.graph.add_node(
                        node_id,
                        bldg_name=bldg_name,
                        floor_id=floor_id,
                        elev_name=elev_name,
                        approach=e_data['approach_pose'],
                        exit=e_data['exit_initial_pose']
                    )
                    self.elevators_by_floor[floor_key].append(node_id)

        # Connect identical elevator shafts vertically across floors
        nodes = list(self.graph.nodes(data=True))
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                n1_id, n1_data = nodes[i]
                n2_id, n2_data = nodes[j]
                
                if (n1_data['bldg_name'] == n2_data['bldg_name']) and (n1_data['elev_name'] == n2_data['elev_name']):
                    self.graph.add_edge(n1_id, n2_id, weight=45.0, action="elevator")
                    self.graph.add_edge(n2_id, n1_id, weight=45.0, action="elevator")

    def get_route(self, start_bldg, start_floor, start_pose, target_bldg, target_floor, target_pose):
        """Calculates optimal sequence of waypoints and map-swap instructions using int floor IDs."""
        start_floor = int(start_floor)
        target_floor = int(target_floor)
        
        # 1. SAME FLOOR SHORT-CIRCUIT
        if start_bldg == target_bldg and start_floor == target_floor:
            return [{
                "action": "navigate", 
                "pose": target_pose,
                "description": f"Navigate directly to goal on Floor {start_floor}"
            }]

        # 2. DIFFERENT FLOOR GRAPH ROUTING
        start_floor_key = f"{start_bldg}_{start_floor}"
        target_floor_key = f"{target_bldg}_{target_floor}"

        self.graph.add_node("START", approach=start_pose)
        self.graph.add_node("GOAL", approach=target_pose)

        # Link START to elevators on current floor
        for elev_node in self.elevators_by_floor.get(start_floor_key, []):
            elev_pose = self.graph.nodes[elev_node]['approach']
            dist = math.dist(start_pose[:2], elev_pose[:2])
            self.graph.add_edge("START", elev_node, weight=dist, action="navigate_to_elevator")

        # Link elevators on target floor to GOAL
        for elev_node in self.elevators_by_floor.get(target_floor_key, []):
            elev_exit = self.graph.nodes[elev_node]['exit']
            dist = math.dist(elev_exit[:2], target_pose[:2])
            self.graph.add_edge(elev_node, "GOAL", weight=dist, action="navigate_to_goal")

        try:
            path = nx.shortest_path(self.graph, source="START", target="GOAL", weight='weight')
        except nx.NetworkXNoPath:
            self._cleanup_temp_nodes()
            return [{"error": "No valid route found."}]

        # Translate path into executable instructions
        sequence = []
        for i in range(len(path) - 1):
            curr_node = path[i]
            next_node = path[i+1]
            edge_data = self.graph.get_edge_data(curr_node, next_node)
            next_node_data = self.graph.nodes[next_node]

            if edge_data['action'] == "navigate_to_elevator":
                sequence.append({
                    "action": "navigate",
                    "pose": next_node_data['approach'],
                    "description": f"Drive to {next_node_data['elev_name']} on Floor {start_floor}"
                })
            elif edge_data['action'] == "elevator":
                sequence.append({
                    "action": "wait_for_elevator",
                    "shaft_name": next_node_data['elev_name'],
                    "target_floor": next_node_data['floor_id']
                })
                target_floor_yaml = self.config['buildings'][target_bldg]['floors'][next_node_data['floor_id']]
                sequence.append({
                    "action": "swap_map_and_initialize",
                    "new_floor_id": next_node_data['floor_id'],
                    "new_occupancy": target_floor_yaml['maps']['occupancy'],
                    "new_pcd": target_floor_yaml['maps']['pcd'],
                    "initial_pose": next_node_data['exit'],
                    "description": f"Load maps for Floor {next_node_data['floor_id']} and set initial pose"
                })
            elif edge_data['action'] == "navigate_to_goal":
                sequence.append({
                    "action": "navigate",
                    "pose": next_node_data['approach'],
                    "description": "Navigate to final goal"
                })

        self._cleanup_temp_nodes()
        return sequence

    def _cleanup_temp_nodes(self):
        if self.graph.has_node("START"): self.graph.remove_node("START")
        if self.graph.has_node("GOAL"): self.graph.remove_node("GOAL")