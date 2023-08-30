from setuptools import setup

package_name = 'proxy_pkg'

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
    maintainer='roy',
    maintainer_email='roy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proxy1 = proxy_pkg.proxy1_node:main',
            'proxy2 = proxy_pkg.proxy2_node:main',
        ],
    },
)
