from setuptools import setup

package_name = 'turtlesim_cleaner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Must match the Python directory name
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ganesh',
    maintainer_email='ganesh@todo.todo',
    description='ROS2 turtlesim_cleaner scripts',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = turtlesim_cleaner.move:main',
            'rotate = turtlesim_cleaner.rotate:main',
            'move2goal = turtlesim_cleaner.move2goal:main',
        ],
    },
)

