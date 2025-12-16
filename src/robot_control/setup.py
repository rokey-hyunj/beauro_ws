from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: [
            '*.yaml',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyunjong',
    maintainer_email='hyunjong@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "beauro = robot_control.beauro:main",
            "chopsticks = robot_control.chopsticks:main",
            "close_grip = robot_control.close_grip:main",
            "errors = robot_control.errors:main",
            "executor = robot_control.executor:main",
            "flatten2 = robot_control.flatten2:main",
            "go_home = robot_control.go_home:main",
            "open_grip = robot_control.open_grip:main",
            "pick_spoid = robot_control.pick_spoid:main",
            "pick_spoon = robot_control.pick_spoon:main",
            "pipetting = robot_control.pipetting:main",
            "scoop2 = robot_control.scoop2:main",
            "scooping = robot_control.scooping:main",
            "spoon_flatten = robot_control.spoon_flatten:main"
        ],
    },
)
