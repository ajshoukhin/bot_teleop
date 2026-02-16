from setuptools import find_packages, setup
import os

package_name = 'bot_teleop'

# Get a list of all files in the 'launch' directory
launch_files = [os.path.join('launch', f) for f in os.listdir('launch') if os.path.isfile(os.path.join('launch', f))]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),  # Include all launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajshoukhin',
    maintainer_email='ajshoukhin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'bot_teleop_node = bot_teleop.bot_teleop_node:main',
        ],
    },
)
