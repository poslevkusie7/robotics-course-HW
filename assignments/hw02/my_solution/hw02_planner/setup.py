from setuptools import find_packages, setup


setup(
    name='hw02_planner',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hw02_planner']),
        ('share/hw02_planner', ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'matplotlib'],
    zip_safe=True,
    maintainer='HW02 student',
    maintainer_email='student@example.invalid',
    description='HW02 configuration space, PRM and A* planner',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['planner = hw02_planner.node:main'],
    },
)
