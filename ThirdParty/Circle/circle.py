import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # 新增导入渐变色线段
import matplotlib.pyplot as plt

def generate_circular_points(center, radius, num_points):
    """
    在三维空间中的XY平面生成均匀分布的圆周点
    
    参数：
        center (tuple): 圆心坐标 (x, y, z)
        radius (float): 圆的半径
        num_points (int): 要生成的点的数量
    
    返回：
        np.ndarray: 包含三维点的数组，形状为 (num_points, 3)
    """
    # 生成等间隔角度（0 到 2π）
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    
    # 计算XY坐标
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    z = np.full_like(angles, center[2])  # Z坐标保持与圆心相同
    
    # 组合成三维点
    return np.column_stack((x, y, z))

def generate_spiral_points(center, max_radius, num_turns, points_per_turn):
    """
     生成先递增后递减的螺旋轨迹（回到初始点），z轴保持不变
    参数：
        center (tuple): 螺旋起始点 (x, y, z)
        max_radius (float): 最大半径
        num_turns (int): 每个阶段的圈数
        points_per_turn (int): 每圈的控制点数
    返回：
        np.ndarray: 三维坐标数组，形状为 (total_points, 3)
    """
    out_points = int(num_turns * points_per_turn)
    in_points = out_points
    # 递增阶段
    angles_out = np.linspace(0, 2 * np.pi * num_turns, out_points, endpoint=False)
    radii_out = np.linspace(0, max_radius, out_points)
    z_out = np.full_like(angles_out, center[2])  # z轴保持不变
    
    # 递减阶段
    angles_in = np.linspace(2 * np.pi * num_turns, 2 * np.pi * num_turns * 2, in_points, endpoint=False)
    radii_in = np.linspace(max_radius, 0, in_points)
    z_in = np.full_like(angles_in, center[2])  # z轴保持不变
    
    # 组合两阶段
    angles = np.concatenate((angles_out, angles_in))
    radii = np.concatenate((radii_out, radii_in))
    z = np.concatenate((z_out, z_in))
    
    x = center[0] + radii * np.cos(angles)
    y = center[1] + radii * np.sin(angles)
    return np.column_stack((x, y, z))

# 示例用法
if __name__ == "__main__":
    # # 定义圆心 (x, y, z)、半径和点数
    # circle_center = (0.0, 0.0, 1.0)
    # circle_radius = 1.5
    # points_count = 16
    
    # # 生成点
    # points = generate_circular_points(circle_center, circle_radius, points_count)
    
    # # 打印结果
    # print("生成的三维圆周点：")
    # for i, point in enumerate(points):
    #     print(f"点{i+1}: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})")

    # 定义螺旋参数
    spiral_center = (0.0, 0.0, 1.5)
    spiral_max_radius = 1.5
    spiral_turns = 2
    points_per_turn = 16
    
    # 生成螺旋点
    spiral_points = generate_spiral_points(spiral_center, spiral_max_radius, spiral_turns, points_per_turn)
    
    # 打印所有螺旋轨迹点
    print("Spiral trajectory points:")
    for i, point in enumerate(spiral_points):
        print(f"[{point[0]}, {point[1]}, {point[2]}],")
    
    # 创建绘图对象
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 根据点序号构造连续线段
    segments = [spiral_points[i:i+2] for i in range(len(spiral_points)-1)]
    # 使用 Colormap 'viridis' 实现渐变效果
    lc = Line3DCollection(segments, cmap='viridis', norm=plt.Normalize(0, len(segments)))
    lc.set_array(np.arange(len(segments)))  # 按序号上色
    lc.set_linewidth(2)
    ax.add_collection3d(lc)

    # 根据数据范围设置坐标轴范围
    ax.set_xlim(np.min(spiral_points[:,0]), np.max(spiral_points[:,0]))
    ax.set_ylim(np.min(spiral_points[:,1]), np.max(spiral_points[:,1]))
    ax.set_zlim(np.min(spiral_points[:,2]), np.max(spiral_points[:,2]))

    # 设置标签和标题
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Spiral Plot with Gradient')

    plt.colorbar(lc, ax=ax, label='Segment Index')
    
    # 保存图像
    # plt.savefig('/spiral_plot.png')
    plt.show()