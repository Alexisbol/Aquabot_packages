from setuptools import find_packages, setup

package_name = 'py_pathfinding'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','custom_interface'],
    zip_safe=True,
    maintainer='mathijs',
    maintainer_email='mathijs@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pathfinding = py_pathfinding.service_member_function:main',
            'client = py_pathfinding.client_member_function:main',
        ],
    },
)
