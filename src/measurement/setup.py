from setuptools import find_packages, setup

package_name = 'measurement'

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
            "z300_measure_asvr = measurement.z300_measure_asvr:main",
            "z300_export_asvr = measurement.z300_export_asvr:main",
            "z300_analyze_asvr = measurement.z300_analyze_asvr:main",
            "is_analytical_dev_rdy_srv = measurement.is_analytical_dev_rdy_srv:main",
        ],
    },
)
