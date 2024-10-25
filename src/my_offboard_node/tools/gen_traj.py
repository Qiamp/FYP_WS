import numpy as np

# 设定四个目标点 (x, y, z)
points = np.array([
    [0.0, 0.0, 0.5],
    [0.0, 0.0, 1.0],
    [1.5, 0.0, 1.0],
    [1.5, 0.0, 0.0]
])


# 插值参数
points_per_second = 100
speed = 0.2  # 每秒移动0.5米

# 计算每两个点之间的距离
distances = np.linalg.norm(points[1:] - points[:-1], axis=1)

# 计算每两个点之间所需的时间
total_time = distances / speed

# 初始化时间序列
t_values = []
interpolated_points = []

# 开始插值
current_time = 0.0
for i in range(len(points) - 1):
    start = points[i]
    end = points[i + 1]
    delta_t = 1.0 / points_per_second  # 每0.01秒插值一个点
    num_interpolated_points = int(total_time[i] * points_per_second)
    
    for j in range(num_interpolated_points):
        t = current_time + j * delta_t
        interp_point = start + (end - start) * (j / num_interpolated_points)
        t_values.append(t)
        interpolated_points.append(interp_point)
    
    current_time += total_time[i]

# 默认的四元数表示（假设不进行旋转）
q_w, q_x, q_y, q_z = 1.0, 0.0, 0.0, 0.0

# 输出文件
output_file = "/home/jay/FYP_WS/src/my_offboard_node/traj/p2p.txt"

with open(output_file, "w") as file:
    for i in range(len(t_values)):
        t = t_values[i]
        x, y, z = interpolated_points[i]
        file.write(f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} {q_w:.6f} {q_x:.6f} {q_y:.6f} {q_z:.6f}")
        file.write(" 0.0" * 16 + "\n")

print(f"插值文件已生成: {output_file}")

