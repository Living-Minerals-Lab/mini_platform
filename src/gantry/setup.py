from setuptools import find_packages, setup

package_name = 'gantry'

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
    maintainer='xcao',
    maintainer_email='caoxuan8872@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_template = gantry.template_node:main",
            "gantry_sub = gantry.gantry_sub:main",
            "gantry_pub = gantry.gantry_pub:main",
            "move_gantry_srv = gantry.move_gantry_srv:main",
            "is_gantry_rdy_srv = gantry.is_gantry_rdy_srv:main",
            "ob_gantry_controller = gantry.openbuilds_gantry_controller:main",
            "set_zero_srv = gantry.set_zero_srv:main"
        ],
    },
)
