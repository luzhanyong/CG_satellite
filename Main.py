import numpy as np

from path.add_ground_column import add_ground_column
from path.add_sat_column import add_sat_column
from path.create_constellation import create_constellation
from path.create_path_ground import create_path_ground
from path.create_path_sat import create_path_sat
from path.initialize_part_path import initialize_part_path
from path.master_solver import master_solver

# Assuming functions like create_constellation, initialize_part_path, etc., are already defined as in your earlier code.

# Define the parameters for the constellation
num_orbit = 6
num_sat_orbit = 5

# 和地面站有连接的卫星数量
num_ground_sat = 6
capacity_sat = 5
capacity_ground = 1
computer_capacity = 10

# Create the satellite constellation
sat_position, adj_matrix_sat, adj_matrix_ground, capacity_matrix, demand_matrix, compute_matrix = create_constellation(
    num_orbit, num_sat_orbit, num_ground_sat, capacity_sat, capacity_ground, computer_capacity)

# Define max hops for satellites and ground
max_hop_sat = 4
max_hop_ground = 4

# Initialize part paths
# 根据初始化的卫星星座的邻接矩阵，找到从一个卫星到另一个卫星的所有路径part_sat_path   和卫星到地面站的所有路径part_ground_path
part_sat_path, part_num_sat_path, part_ground_path, part_num_ground_path = initialize_part_path(
    adj_matrix_sat, adj_matrix_ground, max_hop_sat, max_hop_ground)


print(part_sat_path)

# Create satellite paths
satpath_edges, satpath_source, satpath_terminal = create_path_sat(part_sat_path, part_num_sat_path, adj_matrix_sat)

# Create ground paths
groundpath_edges, groundpath_lastsecond, groundpath_source = create_path_ground(part_ground_path, part_num_ground_path, adj_matrix_ground)

# Define the objective weight vector
obj_weight = np.array([0.5, 0.3, 0.2])

# 使用求解器求出分配给本地、其他卫星、地面站的任务量，并返回约束的对偶
# Call the master solver to get the optimization results
compute_vol, SatCapCon_dual, GroundCapCon_dual, DemandCon_dual, ComputeCon_dual = master_solver(
    capacity_matrix, demand_matrix, compute_matrix, obj_weight,
    satpath_edges, satpath_source, satpath_terminal,
    groundpath_edges, groundpath_lastsecond, groundpath_source)

print(compute_vol)



# Add satellite column to the solution
is_satopt, part_sat_path, part_num_sat_path = add_sat_column(
    adj_matrix_sat, max_hop_sat, part_sat_path, part_num_sat_path,
    SatCapCon_dual, DemandCon_dual, ComputeCon_dual, obj_weight)



# Add ground column to the solution
is_groundopt, part_ground_path, part_num_ground_path = add_ground_column(
    adj_matrix_ground, max_hop_ground, part_ground_path, part_num_ground_path,
    SatCapCon_dual, GroundCapCon_dual, DemandCon_dual, obj_weight)


print(is_groundopt)
print(part_ground_path)
print(part_num_ground_path)