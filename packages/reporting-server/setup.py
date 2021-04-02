from setuptools import setup

package_name = "reporting_server"

setup(
    name=package_name,
    description="RMF api server",
    version="0.0.0",
    packages=["rest_server"],
    author="Matias Bavera",
    author_email="matiasbavera@gmail.com",
    keywords=["ROS", "RMF", "reporting"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    entry_points={
        "console_scripts": [
            "rmf_reporting_server=rest_server.__main__:main",
        ],
    },
    license="Apache License, Version 2.0",
)
