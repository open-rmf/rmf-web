from setuptools import setup

package_name = "api_server"

setup(
    name=package_name,
    description="RMF api server",
    version="0.0.0",
    packages=["api_server", "cli_client"],
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
        "fastapi~=0.63.0",
        "aiofiles~=0.6.0",
        "uvicorn[standard]~=0.13.4",
        "python-socketio~=5.1",
        "rx~=3.1",
        "tortoise-orm~=0.16.21",
        "pyjwt[crypto]~=2.0",
        "pydantic~=1.8",
    ],
    entry_points={
        "console_scripts": [
            "rmf_api_server=api_server.__main__:main",
            "rmf_api_cli=cli_client.__main__:main",
        ],
    },
    license="Apache License, Version 2.0",
)
