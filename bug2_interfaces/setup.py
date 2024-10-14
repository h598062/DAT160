from setuptools import setup

package_name = 'bug2_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='598062@stud.hvl.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bug2_interfaces = bug2_interfaces.bug2_interfaces:main'
        ],
    },
)
