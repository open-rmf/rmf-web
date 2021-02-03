from setuptools import setup

package_name = 'api_server'

setup(
    name=package_name,
    description='RMF api server',
    version='0.0.0',
    packages=['api_server'],
    install_requires=[
        'fastapi>=0.63,<0.64',
        'uvicorn[standard]>=0.13,<0.14',
        'aiofiles>=0.6,<0.7',
        'python-socketio>=5,<6',
        'rx>=3,<4',
        'tortoise-orm>=0.16,<0.17',
    ],
    author='Morgan Quigley',
    author_email='morgan@osrfoundation.org',
    maintainer='Morgan Quigley',
    maintainer_email='morgan@osrfoundation.org',
    keywords=['ROS', 'RMF'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    entry_points={
        'console_scripts': [
            'rmf_api_server=api_server.__main__:main',
        ],
    },
    license='Apache License, Version 2.0',
)
