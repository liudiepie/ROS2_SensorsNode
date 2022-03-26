from setuptools import setup

package_name = 'data_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cliu',
    maintainer_email='cliu@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tempSub = data_processor.tempSub:main',
            'speedSub = data_processor.speedSub:main',
            'laserSub = data_processor.laserSub:main'
        ],
    },
)
