from setuptools import setup

package_name = 'rb_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rb_modbus_bridge.launch.py',
                                                'launch/motion_gui_runner.launch.py',
                                                'launch/motion_gui_runner_chunked.launch.py',
                                                'launch/rb_test_all.launch.py']),
    ],
    install_requires=['setuptools', 'multipledispatch', 'transforms3d', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='RB Test Package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motion_executor = rb_test.motion_executor:main',
            'motion_executor_chunked = rb_test.motion_executor_chunked:main',
            'test_cobot_publisher = rb_test.test_cobot_publisher:main',
            'sensor_gui = rb_test.sensor_gui:main',
            'rb_io_bridge = rb_test.rb_io_bridge:main', 
            'rb_io_bridge_client = rb_test.rb_io_bridge_client:main',
            'rb_modbus_bridge = rb_test.rb_modbus_bridge:main',
            'motion_gui_runner = rb_test.motion_gui_runner:main',
            'motion_yaml_splitter = rb_test.motion_yaml_splitter:main',
        ],
    },
)

