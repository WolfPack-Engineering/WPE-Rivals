from setuptools import setup

package_name = 'pi_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'pi_test_listener = pi_test.pi_test_listener:main',
            'pi_test_sender = pi_test.pi_test_sender:main',
        ],
    },
)