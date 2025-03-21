import numpy as np


def create_path_sat(sat_path, num_sat_path, adj_matrix_sat):

    # 卫星的数量
    num_sat = adj_matrix_sat.shape[0]

    # Initialize the output matrices
    # 给路径编号，一共有num_sat_path个路径，1表示第num_sat_path个路径存在
    satpath_edges = np.zeros((num_sat, num_sat, num_sat_path))
    satpath_source = np.zeros((num_sat, num_sat_path))
    satpath_terminal = np.zeros((num_sat, num_sat_path))

    path_id = 0

    # Loop over each pair of source and terminal satellites
    for source_sat in range(num_sat):
        for terminal_sat in range(num_sat):

            path_ij = sat_path[source_sat][terminal_sat]

            if path_ij is not None:
                num_path_ij = len(path_ij)
            else:
                num_path_ij = 0  # Or some other value indicating no path

            if num_path_ij > 0:
                for s in range(num_path_ij):
                    # Get one path
                    path1 = path_ij[s]
                    path_length = len(path1)

                    # Update the satpath_edges matrix
                    for q in range(path_length - 1):
                        first_node = path1[q]
                        second_node = path1[q + 1]
                        satpath_edges[first_node, second_node, path_id] = 1

                    # Update the satpath_source and satpath_terminal matrices
                    satpath_source[source_sat, path_id] = 1
                    satpath_terminal[terminal_sat, path_id] = 1

                    # Update the path_id
                    path_id += 1

    return satpath_edges, satpath_source, satpath_terminal
