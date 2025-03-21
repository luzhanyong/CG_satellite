import networkx as nx
import numpy as np


def add_ground_column(adj_matrix_ground, max_hop_ground, part_ground_path, part_num_ground_path,
                      SatCapCon_dual, GroundCapCon_dual, DemandCon_dual, obj_weight):
    num_sat_ground = adj_matrix_ground.shape[0]
    num_sat = num_sat_ground - 1

    # Create the graph from adjacency matrix
    graph_map = nx.from_numpy_array(adj_matrix_ground)

    is_groundopt = 1

    # Iterate over each source satellite
    for source_sat in range(1, num_sat + 1):
        # Get all paths from source_sat with max_hop_ground limit
        all_paths_i = list(nx.all_simple_paths(graph_map, source_sat, 0, cutoff=max_hop_ground))
        num_path_i = len(all_paths_i)

        if num_path_i >= 1:
            shortest_length = 10 ** 9
            shortest_path = []

            for path_id in range(num_path_i):
                path__i_j = all_paths_i[path_id]
                sum_dual = 0
                path_length = len(path__i_j)

                if path_length > 2:
                    for q in range(path_length - 2):
                        first_node = path__i_j[q] - 1
                        second_node = path__i_j[q + 1] - 1
                        sum_dual += SatCapCon_dual[first_node, second_node]

                sum_dual += DemandCon_dual[path__i_j[0] - 1]
                sum_dual += GroundCapCon_dual[path__i_j[path_length - 1] - 1]

                # Update the shortest path
                if sum_dual <= shortest_length:
                    shortest_length = sum_dual
                    shortest_path = path__i_j

            # Check if this path violates the dual constraints
            if shortest_length < obj_weight[2]:
                part_ground_path[source_sat - 1].append(shortest_path)
                is_groundopt = 0
                part_num_ground_path += 1

    return is_groundopt, part_ground_path, part_num_ground_path
