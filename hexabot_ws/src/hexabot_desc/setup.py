from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hexabot_desc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Include mesh files only (flat structure)
        (os.path.join('share', package_name, 'urdf', 'meshes'), glob('urdf/meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jpm',
    maintainer_email='cs1221722@iitd.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    # optional: change this to extras_require to avoid warning
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
