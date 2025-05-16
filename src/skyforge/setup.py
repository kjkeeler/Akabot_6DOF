from setuptools import find_packages, setup
from glob import glob

package_name = 'skyforge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Index resource for ROS2 package
        ('share/ament_index/resource_index/packages', ['resource/skyforge']),

        # Install package.xml for ROS2 package recognition
        ('share/skyforge', ['package.xml']),

        # Install launch file
        ('share/skyforge/launch', ['launch/view_robot.launch.py']),

        # Install URDF files
        ('share/skyforge/urdf', ['urdf/system_arch_robot.urdf.xacro']),

        # Install mesh files
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),  # ✅ Correct glob pattern for STL files
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkeeler',
    maintainer_email='kkeeler@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test = skyforge.my_first_node:main"
        ],
    },
)
