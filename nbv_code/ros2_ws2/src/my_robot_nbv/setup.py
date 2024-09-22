from setuptools import find_packages, setup

package_name = 'my_robot_nbv'

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
    maintainer_email='jajajaja4646@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #建立一個excecutable 
            "nbv_tompcd_filter = my_robot_nbv.nbv_tompcd_filter:main"
        ],
    },
    # package_data={ #好像沒功能
    #     'my_robot_nbv': ['module_ICP.py'],  # Include your module here
    # }
)
