from stl import mesh
import numpy as np

def scale_stl(input_file, output_file, scale_factor):
    # 读取STL文件
    your_mesh = mesh.Mesh.from_file(input_file)
    
    # 对每个点的坐标进行缩放
    your_mesh.vectors = your_mesh.vectors * scale_factor
    
    # 保存缩放后的STL文件
    your_mesh.save(output_file)
    print(f"STL file scaled by a factor of {scale_factor} and saved as {output_file}")

if __name__ == "__main__":
    # 输入的STL文件路径
    input_file = '/home/ccwss/PersonalData/Program/Ros2/multi-axle-all-wheel-steering-vehicles_ws/install/multi_axle_vehicle_model/share/multi_axle_vehicle_model/meshes/stl/string.stl'
    # 输出的缩放后STL文件路径
    output_file = '/home/ccwss/PersonalData/Program/Ros2/multi-axle-all-wheel-steering-vehicles_ws/install/multi_axle_vehicle_model/share/multi_axle_vehicle_model/meshes/stl/string1.stl'
    # 缩放因子
    scale_factor = 13
    
    # 调用缩放函数
    scale_stl(input_file, output_file, scale_factor)
