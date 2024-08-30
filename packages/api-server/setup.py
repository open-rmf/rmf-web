from setuptools import find_packages, setup

package_name = "api_server"

setup(
    name=package_name,
    description="Open-RMF API server",
    version="0.2.0",
    packages=find_packages(exclude=["tests"]),
    author="Teo Koon Peng",
    author_email="koonpeng@openrobotics.org",
    keywords=["ROS", "RMF"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    install_requires=[
        "fastapi~=0.109.0",
        "aiofiles~=23.2.1",
        "uvicorn[standard]~=0.28.0",
        "python-socketio~=5.11.1",
        "reactivex~=4.0.4",
        "tortoise-orm~=0.21.4",
        "pyjwt[crypto]~=2.8.0",
        "pydantic~=2.8.0",
        "schedule~=1.2.1",
        "termcolor~=2.4.0",
    ],
    extras_require={
        "postgres": ["asyncpg~=0.29.0"],
        "mysql": ["aiomysql~=0.1.1"],
        "maria": ["aiomysql~=0.1.1"],
    },
    entry_points={
        "console_scripts": [
            "rmf_api_server=api_server.__main__:main",
        ],
    },
    license="Apache License, Version 2.0",
)
