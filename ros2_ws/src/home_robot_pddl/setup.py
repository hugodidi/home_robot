from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'home_robot_pddl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'pddl'),
            glob(os.path.join('pddl', '*.pddl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='hugo.didi.contacto@gmail.com',
    description='PDDL patrol with POPF replanning for Home Robot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'pddl_patrol = home_robot_pddl.pddl_patrol:main',
            'generate_patrol_problem = home_robot_pddl.problem_generator:main',
            'pddl_ui = home_robot_pddl.pddl_ui_server:main',
        ],
    },
)
