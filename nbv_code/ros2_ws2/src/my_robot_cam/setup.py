from setuptools import find_packages, setup

package_name = 'my_robot_cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jajayu',
    maintainer_email='jajayu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

		    #建立一個excecutable 
        "cam_point_cloud_publisher = my_robot_cam.cam_point_cloud_publisher:main",
        "cam_cloudrate_transformer = my_robot_cam.cam_cloudrate_transformer:main"
        ],
    },
)
