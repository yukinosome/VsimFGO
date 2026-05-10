from setuptools import setup

package_name = 'analyse'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yukinosome@qq.com',
    description='Offline trajectory error analysis tools for gnssFGO CSV logs.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'error_analysis = analyse.error_analysis_node:main',
            'position_analysis = analyse.position_analysis_node:main',
        ],
    },
)
