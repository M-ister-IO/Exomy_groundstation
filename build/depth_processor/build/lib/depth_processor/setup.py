from setuptools import setup

package_name = 'depth_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/depth_processor_launch.py']),  # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='charles',
    maintainer_email='charles@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'process_depth = depth_processor.process_depth:main',
            'mapping = depth_processor.mapping:main',
        ],
    },
)