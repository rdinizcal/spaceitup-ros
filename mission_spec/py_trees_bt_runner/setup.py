from setuptools import find_packages, setup

package_name = 'py_trees_bt_runner'

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
    maintainer='gianluca filippone',
    maintainer_email='gianluca.filippone@gssi.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_runner = py_trees_bt_runner.bt_runner_node:main',
            'bt_runner_dynamic = py_trees_bt_runner.bt_runner_node_dynamic:main'
        ],
    },
)
