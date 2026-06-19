from setuptools import find_packages, setup

package_name = 'matias_control'

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
    maintainer='matias',
    maintainer_email='matias@todo.todo',
    description='ROS2 driver for PCA9685 car control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdvel_to_pca9685 = matias_control.cmdvel_to_pca9685:main',
	    'nodo_vision       = matias_control.nodo_vision:main',
            'test_pub          = matias_control.test_publisher:main',
	    'nodo_linea        = matias_control.nodo_linea:main',
            'nodo_canny	       = matias_control.nodo_canny:main',
            'control_teclado   = matias_control.control_teclado:main',
        ],
    },
)
