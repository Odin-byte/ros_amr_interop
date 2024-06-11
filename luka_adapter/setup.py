from setuptools import setup, find_packages

package_name = 'luka_adapter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Loy van Beek',
    maintainer_email='loy.vanbeek@4am-robotics.de',
    description='Adapter to connect 4AM Roboticss LUKA robot to VDA5050_controller',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'luka_adapter = luka_adapter.luka_adapter:main'
        ],
    },
)
