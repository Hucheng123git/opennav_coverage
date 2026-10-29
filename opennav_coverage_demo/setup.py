from glob import glob
import os

from setuptools import setup


package_name = 'opennav_coverage_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='stevenmacenski@gmail.com',
    description='Editable polygon maps and coverage planning visualization',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
                'edit_coverage_map = opennav_coverage_demo.edit_coverage_map:main',
                'coverage_map_client = opennav_coverage_demo.coverage_map_client:main',
        ],
    },
)
