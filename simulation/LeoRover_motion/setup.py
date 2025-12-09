from setuptools import find_packages, setup

package_name = 'leo_rover_fake_motion'

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
    maintainer='student45',
    maintainer_email='student45@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'fake_odom_tf = leo_rover_fake_motion.fake_odom_tf:main',
        'fake_odom_tf_cmdvel = leo_rover_fake_motion.fake_odom_tf_cmdvel:main',

        ],
    },
)
