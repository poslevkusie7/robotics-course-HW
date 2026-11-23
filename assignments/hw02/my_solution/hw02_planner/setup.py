from setuptools import setup

package_name = 'hw02_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HW02 student',
    maintainer_email='student@example.com',
    description='Configuration space, PRM and A* planning for HW02',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['planner_node = hw02_planner.planner_node:main'],
    },
)
