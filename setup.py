from setuptools import find_packages, setup

package_name = 'agx_arm_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gui_params.yaml', 'config/gui_secrets.example.yaml']),
        ('lib/' + package_name, ['scripts/agx_arm_gui', 'scripts/spb_bridge_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='AGX Arm Control GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agx_arm_gui = agx_arm_gui.main:main',
            'spb_bridge_node = agx_arm_gui.spb_bridge_node:main',
        ],
    },
)
