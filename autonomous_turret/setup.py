from setuptools import setup

package_name = 'autonomous_turret'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/autonomous_turret_launch.py']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',   # OpenCV kütüphanesi
        'cv_bridge',       # ROS 2 için OpenCV bağlantısı
    ],
    zip_safe=True,
    maintainer='doggan',
    maintainer_email='your.email@example.com',
    description='Görüntü İşleme ve PID Kontrollü Otonom Taret Sistemi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking_node = autonomous_turret.tracking_node:main',
        ],
    },
)
