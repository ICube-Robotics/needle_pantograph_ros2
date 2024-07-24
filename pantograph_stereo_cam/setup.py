from setuptools import find_packages, setup

package_name = 'pantograph_pantograph_stereo_cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='telecom',
    maintainer_email='lfmaldon@ens2m.org',
    description='Stereo camera package for tracking of the needle tip',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_tracker = pantograph_pantograph_stereo_cam.stereo_tracker:main',
        ],
    },
)
