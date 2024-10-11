from setuptools import setup

package_name = 'sub1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='mgko@morai.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception= sub1.perception:main',
            'controller = sub1.controller:main',
            'publisher=sub1.publisher:main',
            'subscriber=sub1.subscriber:main',
            'handcontrol=sub1.handcontrol:main',
            'odom = sub1.odom:main',
            'make_path = sub1.make_path:main',
            'path_pub = sub1.path_pub:main',
            'path_tracking = sub1.path_tracking:main',
            'path_tracking_ocr = sub1.path_tracking_ocr:main',
            
            'a_star = sub1.a_star:main',
            'load_map = sub1.load_map:main',
            'goal_publisher = sub1.goal_publisher:main',
            'run_localization = sub1.run_localization:main',
            'run_mapping = sub1.run_mapping:main',
            'picture = sub1.picture:main',
            'picture_ocr = sub1.picture_ocr:main',
            'charge_command = sub1.charge_command:main',
            'testapi = sub1.testapi:main', 
            'message_producer = sub1.message_producer:main', 
            'a_star_goal = sub1.a_star_goal:main',
            'charge_ing = sub1.charge_ing:main', 
            'picture_ocr_before = sub1.picture_ocr_before:main'

        ],
    },
)
