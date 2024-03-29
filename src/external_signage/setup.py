#!/usr/bin/env python3

import os

from setuptools import setup


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


package_name = "external_signage"
setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/resource/td5_file", package_files("resource/td5_file")),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/external_signage.launch.xml"]),
    ],    
    install_requires=["setuptools"],
    zip_safe=True,
    author="Makoto Yabuta",
    maintainer="Makoto Yabuta",
    maintainer_email="makoto.yabuta@tier4.jp",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description=("external_signage provides a GUI for passanger."),
    license="TODO",
    entry_points={
        "console_scripts": [
            "external_signage = external_signage.external_signage:main",
        ]
    },
)
