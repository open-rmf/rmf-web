from setuptools import find_packages, setup

package_name = "task_scheduler"

setup(
    name=package_name,
    description="RMF task scheduler",
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    author="Matias Bavera",
    author_email="matiasbavera@gmail.com",
    keywords=["RMF", "task-scheduler"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    install_requires=[
        "fastapi~=0.65.2",
        "uvicorn[standard]~=0.13.4",
        "tortoise-orm~=0.17.4",
        "pyjwt[crypto]~=2.0",
        "python-socketio~=5.1",
        "rx~=3.1",
        "pydantic~=1.8",
        "apscheduler~=1.14.0",
    ],
    extras_require={
        "postgres": ["asyncpg~=0.22.0"],
        "mysql": ["aiomysql~=0.0.21"],
        "maria": ["aiomysql~=0.0.21"],
    },
    entry_points={
        "console_scripts": [
            "task_scheduler=task_scheduler.__main__:main",
        ],
    },
    license="Apache License, Version 2.0",
)
