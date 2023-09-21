from setuptools import setup

package_name = 'g_arm_python_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'g_arm_python_examples.g_arm_python_examples_lib.g_arm_pymoveit2'        
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
        'goal = g_arm_python_examples.goal:main',
        'circular_motion = g_arm_python_examples.circular_motion:main'   
        ],
    },
)
