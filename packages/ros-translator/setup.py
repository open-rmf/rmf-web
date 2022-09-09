from setuptools import find_packages, setup

setup(
    name="ros_translator",
    description="Translates ros interface definitions to other formats",
    version="0.0.1",
    packages=find_packages(exclude=["tests"]),
    package_data={
        "ros_translator": ["typescript/templates/*", "pydantic/*.j2"],
    },
    author="Teo Koon Peng",
    author_email="koonpeng@openrobotics.org",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    install_requires=[
        "jinja2~=3.1.2",
    ],
    entry_points={
        "console_scripts": [
            "ros_translator=ros_translator.__main__:main",
        ],
    },
    license="Apache License, Version 2.0",
)
