from setuptools import find_packages, setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Votre Nom',
    maintainer_email='votre.email@example.com',
    description='Un package ROS 2 pour écouter les messages d odométrie',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking = tracking.tracking:main',
        ],
    },
)
