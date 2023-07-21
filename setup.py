from setuptools import setup

package_name = 'my_franka_emika'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/my_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hci',
    maintainer_email='hci@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "trajectory_publisher = my_franka_emika.trajectory_publisher:main",
            "gripper_publisher = my_franka_emika.gripper_controller:main",
            "app = my_franka_emika.app:main",
            "image_publisher = my_franka_emika.image_publisher:main"
        ],
    },
)
