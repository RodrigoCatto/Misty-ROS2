from setuptools import setup
import os
from glob import glob

package_name = 'misty_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ADDED
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
         # END ADDED
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aimsjetson',
    maintainer_email='aimsjetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'misty_nav = misty_navigation.misty_nav:main',
        ],
    },
)
