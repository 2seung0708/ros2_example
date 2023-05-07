from setuptools import setup

package_name = 'topic_helloworld'

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
    maintainer='shlee',
    maintainer_email='shlee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'helloworld_subscriber = topic_helloworld.helloworld_subscriber:main',
        'helloworld_publisher = topic_helloworld.helloworld_publisher:main',
        ],
    },
)
