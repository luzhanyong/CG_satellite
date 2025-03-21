import networkx as nx
import numpy as np


def add_sat_column(adj_matrix_sat, max_hop_sat, part_sat_path, part_num_sat_path,
                   SatCapCon_dual, DemandCon_dual, ComputeCon_dual, obj_weight):
    num_sat = adj_matrix_sat.shape[0]

    # Create the graph from adjacency matrix
    graph_map = nx.from_numpy_array(adj_matrix_sat)

    is_satopt = 1

    # Iterate over each pair of source and terminal satellites
    for source_sat in range(num_sat):
        for terminal_sat in range(num_sat):
            if source_sat != terminal_sat:
                # Get all paths from source_sat to terminal_sat with max_hop_sat limit
                all_paths_i = list(
                    nx.all_simple_paths(graph_map, source=source_sat, target=terminal_sat, cutoff=max_hop_sat))
                num_path_i = len(all_paths_i)

                if num_path_i >= 1:
                    shortest_length = 10 ** 9
                    shortest_path = []

                    for path_id in range(num_path_i):
                        path__i_j = all_paths_i[path_id]
                        sum_dual = 0
                        path_length = len(path__i_j)

                        # Sum the dual values of the edges in the path
                        for q in range(path_length - 1):
                            first_node = path__i_j[q]
                            second_node = path__i_j[q + 1]
                            sum_dual += SatCapCon_dual[first_node, second_node]

                        # Add the dual values for the source and terminal nodes
                        sum_dual += DemandCon_dual[path__i_j[0]]
                        sum_dual += ComputeCon_dual[path__i_j[path_length - 1]]

                        # Update the shortest path if the current path has a smaller dual sum
                        if sum_dual <= shortest_length:
                            shortest_length = sum_dual
                            shortest_path = path__i_j

                    # Check if this path violates the dual constraints
                    if shortest_length < obj_weight[1]:
                        part_sat_path[source_sat][terminal_sat].append(shortest_path)
                        is_satopt = 0
                        part_num_sat_path += 1

    return is_satopt, part_sat_path, part_num_sat_path
