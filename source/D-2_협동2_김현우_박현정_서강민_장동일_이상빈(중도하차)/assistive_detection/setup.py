from glob import glob
from setuptools import find_packages, setup

package_name = 'assistive_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name, ['README.md', 'requirements.txt']),
    ],
    package_data={
        package_name: ['*.npy', '*.pt'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='rokey',
    maintainer_email='rokey@example.com',
    description='Assistive robot hand detection, hand tracking, and object detection nodes.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'hand_detection = assistive_detection.hand_detection:main',
            'hand_tracking = assistive_detection.hand_tracking:main',
            'object_detection = assistive_detection.object_detection:main',
        ],
    },
)
