from setuptools import setup

package_name = 'fgo_logger'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yukinosome@qq.com',
    description='Subscribe online_fgo state and log raw ECEF and converted LLH to CSV.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csv_logger = fgo_logger.state_csv_logger_node:main',
        ],
    },
)
