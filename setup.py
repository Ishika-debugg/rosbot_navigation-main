from setuptools import find_packages, setup

package_name = 'rosbot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/rosbot_navigation']),
    ('share/rosbot_navigation', ['package.xml']),
    ('share/rosbot_navigation/launch', ['launch/tracker_test.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = rosbot_navigation.navigation_logic:main'
            'position_tracker_node = rosbot_navigation.position_tracker_node:main',
        ],
    },
)
