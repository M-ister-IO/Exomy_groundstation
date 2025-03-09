from setuptools import find_packages, setup

package_name = 'depth_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='charles',
    maintainer_email='mcpccharles@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'process_depth = depth_processor.scripts.process_depth:main',
            'mapping = depth_processor.scripts.mapping:main',
        ],
    },
)
