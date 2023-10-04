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
        "fastapi~=0.78.0",
        "aiofiles~=0.8.0",
        "uvicorn[standard]~=0.18.2",
        "python-socketio~=5.7",
        "rx~=3.2",
        "tortoise-orm~=0.18.1",
        "pyjwt[crypto]~=2.4",
        "pydantic~=1.9",
        "schedule~=1.1.0",
    ],
    extras_require={
        "postgres": ["asyncpg~=0.25.0"],
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
