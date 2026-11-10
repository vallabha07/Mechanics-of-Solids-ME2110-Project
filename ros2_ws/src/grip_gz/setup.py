from glob import glob
from setuptools import setup

package_name = 'grip_gz'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='co25btech11020@iith.ac.in',
    description='Mechanics of Solids adaptive grip force control simulation in MuJoCo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'grip_controller = grip_gz.controller_node:main',
            'grip_judge = grip_gz.judge_node:main',
            'grip_dashboard = grip_gz.dashboard:main',
            'grip_world = grip_gz.world:main',
            'grip_materials = grip_gz.materials:main',
            'grip_cad = grip_gz.cad:main',
            'grip_hand = grip_gz.hand_plant:main',
        ],
    },
)
