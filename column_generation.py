import numpy as np

from path.add_ground_column import add_ground_column
from path.add_sat_column import add_sat_column
from path.create_path_ground import create_path_ground
from path.create_path_sat import create_path_sat
from path.initialize_part_path import initialize_part_path
from path.master_solver import master_solver


def column_generation(adj_matrix_sat, adj_matrix_ground, capacity_matrix, demand_matrix, compute_matrix,
                      max_hop_sat, max_hop_ground, obj_weight):
    # Initialize the part of feasible paths
    part_sat_path, part_num_sat_path, part_ground_path, part_num_ground_path = initialize_part_path(
        adj_matrix_sat, adj_matrix_ground, max_hop_sat, max_hop_ground
    )

    # While loop for column generation
    is_opt = False

    while not is_opt:
        print(part_num_ground_path, part_num_sat_path)

        # Create path related matrices
        satpath_edges, satpath_source, satpath_terminal = create_path_sat(part_sat_path, part_num_sat_path, adj_matrix_sat)
        groundpath_edges, groundpath_lastsecond, groundpath_source = create_path_ground(part_ground_path, part_num_ground_path, adj_matrix_ground)

        # Solve the restricted master problem
        compute_vol, SatCapCon_dual, GroundCapCon_dual, DemandCon_dual, ComputeCon_dual = master_solver(
            capacity_matrix, demand_matrix, compute_matrix, obj_weight,
            satpath_edges, satpath_source, satpath_terminal,
            groundpath_edges, groundpath_lastsecond, groundpath_source
        )

        # Add column for satellite paths
        is_satopt, part_sat_path, part_num_sat_path = add_sat_column(
            adj_matrix_sat, max_hop_sat, part_sat_path, part_num_sat_path,
            SatCapCon_dual, DemandCon_dual, ComputeCon_dual, obj_weight
        )

        # Add column for ground paths
        is_groundopt, part_ground_path, part_num_ground_path = add_ground_column(
            adj_matrix_ground, max_hop_ground, part_ground_path, part_num_ground_path,
            SatCapCon_dual, GroundCapCon_dual, DemandCon_dual, obj_weight
        )

        # Check if optimal solution is reached
        is_opt = is_satopt and is_groundopt

    # Calculate the number of active paths
    active_num_satpath = part_num_sat_path
    active_num_groundpath = part_num_ground_path

    return compute_vol, active_num_satpath, active_num_groundpath
