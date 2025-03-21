import numpy as np


def create_path_ground(ground_path, num_ground_path, adj_matrix_ground):
    num_sat_ground = adj_matrix_ground.shape[0]
    num_sat = num_sat_ground - 1

    # Initialize the output matrices
    groundpath_edges = np.zeros((num_sat, num_sat, num_ground_path))
    groundpath_lastsecond = np.zeros((num_sat, num_ground_path))
    groundpath_source = np.zeros((num_sat, num_ground_path))

    path_id = 0

    # Loop over each source satellite (starting from 2 as in MATLAB)
    for source_sat in range(1, num_sat + 1):
        path_ij = ground_path[source_sat]
        num_path_ij = len(path_ij)

        if num_path_ij > 0:
            for s in range(num_path_ij):
                # Get one path
                path1 = path_ij[s]
                path_length = len(path1)

                # Update the groundpath_edges matrix
                if path_length > 2:
                    for q in range(path_length - 2):
                        first_node = path1[q]
                        second_node = path1[q + 1]
                        groundpath_edges[first_node - 1, second_node - 1, path_id] = 1

                # Update the groundpath_source matrix
                groundpath_source[source_sat - 1, path_id] = 1

                # Update the groundpath_lastsecond matrix
                lastsecond_sat_id = path1[path_length - 1]
                groundpath_lastsecond[lastsecond_sat_id - 1, path_id] = 1

                # Increment the path_id
                path_id += 1

    return groundpath_edges, groundpath_lastsecond, groundpath_source
