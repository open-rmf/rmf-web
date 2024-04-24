from setuptools import find_packages, setup

package_name = "api_server"

setup(
    name=package_name,
    description="RMF api server",
    version="0.0.1",
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
    python_requires="~=3.10.4",
    install_requires=[
        "fastapi~=0.109.0",
        "aiofiles~=23.2.1",
        "uvicorn[standard]~=0.28.0",
        "python-socketio~=5.11.1",
        "reactivex~=4.0.4",
        "tortoise-orm~=0.20.0",
        "pyjwt[crypto]~=2.8.0",
        "pydantic~=2.6.4",
        "schedule~=1.2.1",
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
    package_data={
        "api_server.static": ["*.js", "*.css"],
    },
    license="Apache License, Version 2.0",
)
