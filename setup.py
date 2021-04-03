from setuptools import setup

package_name = 'hal'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','tinymovr'],
    zip_safe=True,
    maintainer='SylvainInc',
    maintainer_email='contact@konexinc.com',
    description="""Hardware Abstraction layer for project Vanille. Implement low level, vendor specific
    hardware drivers and provide a standard ros messages interface""",
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vanille_hal = hal.vanille_hal:main',
            'test_cmd = hal.test_cmd:main',
            'test_traj = hal.test_traj:main',
            'inverse_kinematic = hal.inverse_kinematic:main'
        ],
    },
)
