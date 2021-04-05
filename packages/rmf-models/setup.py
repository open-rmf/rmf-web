from setuptools import setup

package_name = "ts_ros"

setup(
    name=package_name,
    description="Generates typescript definitions from ROS messages",
    version="0.0.1",
    packages=["ts_ros"],
    author="Teo Koon Peng",
    author_email="koonpeng@openrobotics.org",
    keywords=["ROS", "typescript"],
    install_requires=[
        "jinja2==2.*",
    ],
    package_data={
        "ts_ros": ["templates/*"],
    },
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    entry_points={
        "console_scripts": [
            "ts_ros=ts_ros.__main__:main",
        ],
    },
    license="Apache License, Version 2.0",
)
