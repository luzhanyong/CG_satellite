import numpy as np

def create_constellation(num_orbit, num_sat_orbit, num_ground_sat, capacity_sat, capacity_ground, computer_capacity):
    num_sat = num_orbit * num_sat_orbit

    # Initialize satellite positions   维数：卫星数*3  第0列是卫星编号、第1列是轨道id、第2列是卫星轨道id
    sat_position = np.zeros((num_sat, 3))
    sat_position[:, 0] = np.arange(1, num_sat + 1)

    sat_id = 0
    for orbit_id in range(1, num_orbit + 1):
        for sat_orbit_id in range(1, num_sat_orbit + 1):
            sat_position[sat_id, 1] = orbit_id
            sat_position[sat_id, 2] = sat_orbit_id
            sat_id += 1

    # Initialize adjacency matrices
    # 卫星之间的临界矩阵
    adj_matrix_sat = np.zeros((num_sat, num_sat))
    # 卫星、地面站之间的临界矩阵，第0列是地面站
    adj_matrix_ground = np.zeros((num_sat + 1, num_sat + 1))

    # Create adjacency matrix for satellites and ground stations
    for sat_id_1 in range(num_sat):
        for sat_id_2 in range(num_sat):
            orbit_id_1 = sat_position[sat_id_1, 1]
            sat_orbit_id_1 = sat_position[sat_id_1, 2]
            orbit_id_2 = sat_position[sat_id_2, 1]
            sat_orbit_id_2 = sat_position[sat_id_2, 2]

            # Check if satellites are in the same orbit
            if orbit_id_1 == orbit_id_2:
                if (abs(sat_orbit_id_1 - sat_orbit_id_2) == 1) or \
                   (sat_orbit_id_1 == 1 and sat_orbit_id_2 == num_sat_orbit) or \
                   (sat_orbit_id_1 == num_sat_orbit and sat_orbit_id_2 == 1):
                    adj_matrix_sat[sat_id_1, sat_id_2] = 1
                    adj_matrix_sat[sat_id_2, sat_id_1] = 1
                    adj_matrix_ground[sat_id_1 + 1, sat_id_2 + 1] = 1
                    adj_matrix_ground[sat_id_2 + 1, sat_id_1 + 1] = 1

            # Check if satellites are in the same position in different orbits
            if sat_orbit_id_1 == sat_orbit_id_2:
                if (abs(orbit_id_1 - orbit_id_2) == 1) or \
                   (orbit_id_1 == 1 and orbit_id_2 == num_orbit) or \
                   (orbit_id_1 == num_orbit and orbit_id_2 == 1):
                    adj_matrix_sat[sat_id_1, sat_id_2] = 1
                    adj_matrix_sat[sat_id_2, sat_id_1] = 1
                    adj_matrix_ground[sat_id_1 + 1, sat_id_2 + 1] = 1
                    adj_matrix_ground[sat_id_2 + 1, sat_id_1 + 1] = 1

    # Ground station adjacency (connected to satellite 1)
    # 地面站与前面num_ground_sat个卫星有连接
    adj_matrix_ground[0, 1:num_ground_sat + 1] = 1
    adj_matrix_ground[1:num_ground_sat + 1, 0] = 1

    # Initialize the capacity matrix
    capacity_matrix = np.zeros((num_sat + 1, num_sat + 1))

    # Ground station capacity
    capacity_matrix[0, 1:num_ground_sat + 1] = capacity_ground
    capacity_matrix[1:num_ground_sat + 1, 0] = capacity_ground

    # Satellite-to-satellite capacity (based on adjacency matrix)
    for sat_id_1 in range(1, num_sat + 1):
        for sat_id_2 in range(1, num_sat + 1):
            if adj_matrix_ground[sat_id_1, sat_id_2] == 1:
                capacity_matrix[sat_id_1, sat_id_2] = capacity_sat

    # Communication demand for each satellite (log-normal distribution)
    com_avg = 20  # Example average communication demand
    com_dev = 1.3  # Standard deviation of the communication demand
    com_mu = np.log(com_avg) - 0.5 * com_dev ** 2
    demand_matrix = np.random.lognormal(mean=com_mu, sigma=com_dev, size=(num_sat, 1))

    # Computational capacity for each satellite
    compute_matrix = computer_capacity * np.ones((num_sat, 1))


    return sat_position, adj_matrix_sat, adj_matrix_ground, capacity_matrix, demand_matrix, compute_matrix
