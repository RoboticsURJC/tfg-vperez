from setuptools import setup

package_name = 'g_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'g_arm.g_arm_lib.robot',
        'g_arm.g_arm_lib.grblAPI'
        
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vidi',
    maintainer_email='v.perezb.2019@alumnos.urjc.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'driver = g_arm.driver:main',
            'example_trajectory_movement = g_arm.examples.example_trajectory_movement:main'
        ],
    },
)
