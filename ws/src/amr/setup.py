from setuptools import find_packages, setup

package_name = 'amr'

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
    maintainer='ll',
    maintainer_email='lukas.lindenroth@kcl.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_interface = amr.hardware_interface:main',
            'hardware_interface_motor_only = amr.hardware_interface_motor_only:main',
            'joint_publisher = amr.joint_publisher:main',
            'joint_rel_publisher = amr.joint_rel_publisher:main',
            'kinematics = amr.kinematics:main',
            'trajectory = amr.trajectory:main',
            'motor_only_traj = amr.motor_only_traj:main',
            'target_pos = amr.target_pos:main',
            'external_torque_reader = amr.external_torque_reader:main',
            'pid_controller = amr.pid_controller:main'

        ],
    },
)
