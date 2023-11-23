from setuptools import find_packages, setup

package_name = 'follow_apriltag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf-transformations'],
    zip_safe=True,
    maintainer='dominik',
    maintainer_email='dominik.nwk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_apriltag = follow_apriltag.follow_apriltag:main',
        ],
    },
)
