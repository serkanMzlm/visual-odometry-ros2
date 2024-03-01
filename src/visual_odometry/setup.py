from glob import glob
from setuptools import find_packages, setup

package_name = 'visual_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name + '/video.py',
                                 package_name + '/image.py',
                                 package_name + '/camera.py',
                                 package_name + '/plotting.py',]),
        ('lib/' + package_name + '/KITTI_sequence_1', glob(package_name + '/KITTI_sequence_1/*.txt')),
        ('lib/' + package_name + '/KITTI_sequence_2', glob(package_name + '/KITTI_sequence_2/*.txt')),
        ('lib/' + package_name + '/KITTI_sequence_1/image_l', glob(package_name + '/KITTI_sequence_1/image_l/*')),
        ('lib/' + package_name + '/KITTI_sequence_1/image_r', glob(package_name + '/KITTI_sequence_1/image_r/*')),
        ('lib/' + package_name + '/KITTI_sequence_2/image_l', glob(package_name + '/KITTI_sequence_2/image_l/*')),
        ('lib/' + package_name + '/KITTI_sequence_2/image_r', glob(package_name + '/KITTI_sequence_2/image_r/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='serkan',
    maintainer_email='serkanmazlum306@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vo_node = visual_odometry.VO_node:main'
        ],
    },
)
