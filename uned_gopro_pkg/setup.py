from setuptools import find_packages, setup

package_name = 'uned_gopro_pkg'

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
    maintainer='kiko',
    maintainer_email='fma527@ual.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stream = uned_gopro_pkg.stream:main',
            'stream_test = uned_gopro_pkg.stream_test:main'
        ],
    },
)
