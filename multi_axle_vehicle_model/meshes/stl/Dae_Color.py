import collada

def colorize_dae(filename, output_filename, color):
    # 加载 DAE 文件
    mesh = collada.Collada(filename)
    
    # 创建一个新的效果和材料来应用颜色
    effect = collada.material.Effect("new_effect", [], "phong", diffuse=color)
    mat = collada.material.Material("new_material", "material1", effect)
    mesh.effects.append(effect)
    mesh.materials.append(mat)
    
    # 为每个几何体的每个原语设置新的材料
    for geom in mesh.geometries:
        for prim in geom.primitives:
            prim.material = mat
    
    # 保存更改后的文件
    mesh.write(output_filename)

# 定义颜色 (r, g, b, a)
color = (1, 0, 0, 1)  # 红色，RGBA格式

# 调用函数
colorize_dae("/home/ccwss/PersonalData/Program/Ros2/multi-axle-all-wheel-steering-vehicles_ws/install/multi_axle_vehicle_model/share/multi_axle_vehicle_model/meshes/dae/car.dae", "/home/ccwss/PersonalData/Program/Ros2/multi-axle-all-wheel-steering-vehicles_ws/install/multi_axle_vehicle_model/share/multi_axle_vehicle_model/meshes/dae/car1.dae", color)
