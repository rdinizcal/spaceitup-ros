from setuptools import find_packages, setup

package_name = 'mars_rover_nav'

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
    maintainer='Gianluca Filippone',
    maintainer_email='gianluca.filippoe@gssi.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_nav = mars_rover_nav.mock_navigation_server:main',
            'mars_rover_nav = mars_rover_nav.mars_rover_navigation_server:main'
        ],
    },
)