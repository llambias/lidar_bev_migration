from setuptools import find_packages, setup

package_name = 'bird_eye_view_2'

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
    maintainer='agustin',
    maintainer_email='agustn.llambias@uc.cl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bird_eye_view = bird_eye_view_2.bird_eye_view:main'
        ],
    },
)
