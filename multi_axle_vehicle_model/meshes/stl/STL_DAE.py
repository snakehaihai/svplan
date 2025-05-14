import trimesh

def convert_stl_to_dae(input_stl, output_dae):
    try:
        # 加载STL文件
        mesh = trimesh.load(input_stl)
        
        # 将模型导出为DAE文件
        mesh.export(output_dae)
        
        print(f"成功将 {input_stl} 转换为 {output_dae}")
    
    except Exception as e:
        print(f"转换失败: {e}")

if __name__ == "__main__":
    # 替换为你自己的STL文件路径
    input_stl_file = "/home/ccwss/PersonalData/Program/Ros2/multi-axle-all-wheel-steering-vehicles_ws/install/multi_axle_vehicle_model/share/multi_axle_vehicle_model/meshes/stl/car.stl"
    
    # 指定导出的DAE文件路径
    output_dae_file = "/home/ccwss/PersonalData/Program/Ros2/multi-axle-all-wheel-steering-vehicles_ws/install/multi_axle_vehicle_model/share/multi_axle_vehicle_model/meshes/dae/car.dae"
    
    # 调用转换函数
    convert_stl_to_dae(input_stl_file, output_dae_file)
