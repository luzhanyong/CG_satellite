import numpy as np
import gurobipy as gp
from gurobipy import GRB


def master_solver(capacity_matrix, demand_matrix, compute_matrix, obj_weight,
                  satpath_edges, satpath_source, satpath_terminal,
                  groundpath_edges, groundpath_lastsecond, groundpath_source):
    num_sat = satpath_edges.shape[0]
    num_sat_path = satpath_edges.shape[2]
    num_ground_path = groundpath_edges.shape[2]

    # 创建模型
    model = gp.Model("SatelliteNetworkOptimization")
    model.setParam('OutputFlag', 0)  # 关闭求解日志

    # 定义遥感数据的总数据量大小
    D  = np.random.uniform(200, 500)

    # 添加变量
    x_i = model.addVars(num_sat, lb=0, ub=D, name="x_i")




    x_i = model.addVars(num_sat, name="x_i", lb=0.0)  # 卫星本地计算量
    sat_flow = model.addVars(num_sat_path, name="sat_flow", lb=0.0)  # 卫星路径流量
    ground_flow = model.addVars(num_ground_path, name="ground_flow", lb=0.0)  # 地面路径流量

    # 存储约束对象以便获取对偶变量
    SatCapCon = {}
    GroundCapCon = {}
    DemandCon = None
    ComputeCon = None

    # 1. 卫星间链路容量约束
    for i in range(num_sat):
        for j in range(num_sat):
            if capacity_matrix[i + 1, j + 1] > 0:
                # 获取该链路上的所有路径
                sat_paths = [p for p in range(num_sat_path) if satpath_edges[i, j, p] > 0]
                ground_paths = [p for p in range(num_ground_path) if groundpath_edges[i, j, p] > 0]

                if sat_paths or ground_paths:
                    # 创建一个表达式
                    lhs = gp.LinExpr()
                    # 卫星路径贡献
                    for p in sat_paths:
                        lhs += sat_flow[p]
                    # 地面路径贡献
                    for p in ground_paths:
                        lhs += ground_flow[p]
                    # 添加约束
                    SatCapCon[(i, j)] = model.addConstr(
                        lhs <= capacity_matrix[i + 1, j + 1],
                        name=f"SatCapCon_{i}_{j}"
                    )

    # 2. 地面链路容量约束
    for i in range(num_sat):
        if capacity_matrix[i, 0] > 0:
            # 获取所有使用该地面链路的路径
            ground_paths = [p for p in range(num_ground_path) if groundpath_lastsecond[i, p] > 0]
            if ground_paths:
                lhs = gp.LinExpr()
                for p in ground_paths:
                    lhs += ground_flow[p]
                GroundCapCon[i] = model.addConstr(
                    lhs <= capacity_matrix[i, 0],
                    name=f"GroundCapCon_{i}"
                )

    # 3. 需求约束（每个卫星节点一个）
    demand_constrs = []
    for i in range(num_sat):
        expr = x_i[i]
        # 卫星路径源贡献
        for p in range(num_sat_path):
            expr += satpath_source[i, p] * sat_flow[p]
        # 地面路径源贡献
        for p in range(num_ground_path):
            expr += groundpath_source[i, p] * ground_flow[p]
        demand_constrs.append(
            model.addConstr(expr <= demand_matrix[i], name=f"DemandCon_{i}")
        )
    DemandCon = demand_constrs  # 保存为列表

    # 4. 计算能力约束（每个卫星节点一个）
    compute_constrs = []
    for i in range(num_sat):
        expr = x_i[i]
        # 卫星路径终点贡献
        for p in range(num_sat_path):
            expr += satpath_terminal[i, p] * sat_flow[p]
        compute_constrs.append(
            model.addConstr(expr <= compute_matrix[i], name=f"ComputeCon_{i}")
        )
    ComputeCon = compute_constrs

    # 设置目标函数
    obj = gp.LinExpr()
    # 本地计算项
    obj += obj_weight[0] * sum(x_i[i] for i in range(num_sat))
    # 卫星流量项
    obj += obj_weight[1] * sum(sat_flow[p] for p in range(num_sat_path))
    # 地面流量项
    obj += obj_weight[2] * sum(ground_flow[p] for p in range(num_ground_path))
    model.setObjective(obj, GRB.MAXIMIZE)  # 原MATLAB代码是最大化目标

    # 求解模型
    model.optimize()

    # 检查求解状态
    if model.status != GRB.OPTIMAL:
        raise Exception(f"Optimization failed with status {model.status}")

    # 提取结果
    compute_vol = np.zeros(3)
    compute_vol[0] = sum(x_i[i].X for i in range(num_sat))
    compute_vol[1] = sum(sat_flow[p].X for p in range(num_sat_path))
    compute_vol[2] = sum(ground_flow[p].X for p in range(num_ground_path))

    # 提取对偶变量
    SatCapCon_dual = np.zeros((num_sat, num_sat))
    for i in range(num_sat):
        for j in range(num_sat):
            if (i, j) in SatCapCon:
                # SatCapCon卫星容量约束，
                SatCapCon_dual[i, j] = SatCapCon[(i, j)].Pi

    GroundCapCon_dual = np.zeros(num_sat)
    for i in range(num_sat):
        if i in GroundCapCon:
            # GroundCapCon地面容量约束
            GroundCapCon_dual[i] = GroundCapCon[i].Pi

    # 需求约束对偶变量
    DemandCon_dual = np.array([c.Pi for c in DemandCon])
    # 计算约束对偶变量
    ComputeCon_dual = np.array([c.Pi for c in ComputeCon])

    return compute_vol, SatCapCon_dual, GroundCapCon_dual, DemandCon_dual, ComputeCon_dual