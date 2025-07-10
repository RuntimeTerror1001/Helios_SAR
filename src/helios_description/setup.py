from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'helios_description'

# 1. start with the ament resource index entry
data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
]

# 2. walk every file under models/
for root, _, files in os.walk('models'):
    if not files:
        continue
    # compute destination sub-folder under share/helios_description/models
    rel_path = os.path.relpath(root, 'models')
    dest_dir = os.path.join(f'share/{package_name}/models', rel_path)

    # full paths of all files in this folder
    src_files = [os.path.join(root, f) for f in files]
    data_files.append((dest_dir, src_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='redpaladin',
    maintainer_email='parthraj1001@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
