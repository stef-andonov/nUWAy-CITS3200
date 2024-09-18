from setuptools import find_packages, setup

package_name = 'gsml'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'opencv-python'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gsml_publisher_1 = gsml.pub1:main',
            'gsml_publisher_2 = gsml.pub2:main',
            'gsml_publisher_3 = gsml.pub3:main',
            'gsml_publisher_4 = gsml.pub4:main',
            'gsml_subscriber = gsml.sub_separate:main',
            'gsml_calibrate = gsml.sub_separate_calibrate:main',
        ],
    },
)
