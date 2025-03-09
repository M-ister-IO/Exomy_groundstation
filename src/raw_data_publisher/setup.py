from setuptools import setup, find_packages

package_name = 'raw_data_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='charles',
    maintainer_email='charles@example.com',
    description='Raw data publisher for IMU',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = raw_data_publisher.scripts.imu_publisher:main',
            'tof_publisher = raw_data_publisher.scripts.tof_publisher:main',
        ],
    },
)
