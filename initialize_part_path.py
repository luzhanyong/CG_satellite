import networkx as nx

def initialize_part_path(adj_matrix_sat, adj_matrix_ground, max_hop_sat, max_hop_ground):
    num_sat = len(adj_matrix_sat)

    # Create satellite graph from adjacency matrix
    sat_graph = nx.from_numpy_array(adj_matrix_sat)

    # 一个卫星到另一个卫星的所有路径，使用三维数组表示
    sat_path = [[None for _ in range(num_sat)] for _ in range(num_sat)]

    # 卫星星座中所以卫星的数量
    num_sat_path = 0

    # Find all paths between satellite pairs
    for source_sat in range(num_sat):
        for terminal_sat in range(num_sat):
            if source_sat != terminal_sat:
                # Find all paths between source_sat and terminal_sat with max hops = max_hop_sat
                paths_ij = list(nx.all_simple_paths(sat_graph, source_sat, terminal_sat, cutoff=max_hop_sat))
                sat_path[source_sat][terminal_sat] = paths_ij

                num_sat_path += len(paths_ij)

    # Create ground graph from adjacency matrix
    ground_graph = nx.from_numpy_array(adj_matrix_ground)

    ground_path = [None] * (num_sat + 1)
    num_ground_path = 0

    # Find paths from ground stations (2 to num_sat + 1) to satellite 1 (index 0)
    for source_ground in range(1, num_sat + 1):
        # Find all paths from source_ground to satellite 1 (index 0) with max hops = max_hop_ground
        paths_ij = list(nx.all_simple_paths(ground_graph, source_ground, 0, cutoff=max_hop_ground))
        ground_path[source_ground] = paths_ij

        num_ground_path += len(paths_ij)

    return sat_path, num_sat_path, ground_path, num_ground_path
