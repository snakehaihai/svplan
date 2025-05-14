import os
from setuptools import setup

package_name = 'multi_axle_vehicle_model'

def package_files(directory):
    paths = []
    base_path = os.path.abspath(directory)  # 获取绝对路径
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # 生成从package目录开始的相对路径
            rel_path = os.path.relpath(os.path.join(path, filename), base_path)
            paths.append(os.path.join(directory, rel_path))
    return paths

launch_files = package_files('launch')
xacro_files = package_files('xacro')
meshes_dae_files = package_files('meshes/dae')
meshes_stl_files = package_files('meshes/stl')
control_files = package_files('control')
worlds_files = ['worlds/city.world']
rviz_files = package_files('rviz')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/xacro', xacro_files),
        ('share/' + package_name + '/meshes/dae', meshes_dae_files),
        ('share/' + package_name + '/meshes/stl', meshes_stl_files),
        ('share/' + package_name + '/control', control_files),
        ('share/' + package_name + '/worlds', worlds_files),
        ('share/' + package_name + '/rviz', rviz_files),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ccwss',
    maintainer_email='2922520613@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
