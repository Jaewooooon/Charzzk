from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'my_package.odometry',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='SSAFY@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'communication = my_package.communication:main',
            'odometry = my_package.odometry:main',
            'tf_broadcast = my_package.tf_broadcast:main',
            'path=my_package.path:main',
            'path_tracking=my_package.path_tracking:main',
            'collision_check=my_package.collision_check:main',
            'make_path=my_package.make_path:main'
        ],
    },
)
