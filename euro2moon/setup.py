from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'euro2moon'



# All the individual files which needs to be copied to the install directory
data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
    ]

# This custom funciton allows us to recursively copy all the files in the folders to the install directory
def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    # data_files=[
    #     ('share/ament_index/resource_index/packages',['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),

    #     (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
    #     (os.path.join('share', package_name, 'launch', 'include'), glob('launch/include/*.*')),
    #     (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    #     (os.path.join('share', package_name, 'meshes'), glob('meshes/*.*')),
    #     (os.path.join('share', package_name, 'meshes', 'rover'), glob('meshes/rover/*.*')),
    #     (os.path.join('share', package_name, 'meshes', 'map'), glob('meshes/map/*.*')),
    #     (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
    #     (os.path.join('share', package_name, 'urdf', 'rover'), glob('urdf/rover/*.*')),
    #     (os.path.join('share', package_name, 'config'), glob('config/*')),
    # ],

    data_files = package_files(data_files, ['worlds/', 'meshes/', 'urdf/', 'config/']),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas',
    maintainer_email='lucas.naury@sfr.fr',
    description='ROS2 Package for simulating a Moon Rover',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'my_robot_driver = my_package.my_robot_driver:main',
            # 'obstacle_avoider = my_package.obstacle_avoider:main'
            'my_robot_driver = euro2moon.my_robot_driver:main',
            'obstacle_avoider = euro2moon.obstacle_avoider:main',
            'fake_path = euro2moon.fake_path_publisher:main',
            'plotter = euro2moon.trajectory_plotting:main'
        ],
    },
)
