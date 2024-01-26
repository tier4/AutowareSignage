#!/usr/bin/env python3

import os

from setuptools import setup


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


package_name = "signage"
setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/resource/page", package_files("resource/page")),
        ("share/" + package_name + "/resource/sound", package_files("resource/sound")),
        (
            "share/" + package_name + "/resource/page/BusStopView",
            package_files("resource/page/BusStopView"),
        ),
        (
            "share/" + package_name + "/resource/page/BusMoveView",
            package_files("resource/page/BusMoveView"),
        ),
        (
            "share/" + package_name + "/resource/page/EmergencyStopView",
            package_files("resource/page/EmergencyStopView"),
        ),
        (
            "share/" + package_name + "/resource/page/Common",
            package_files("resource/page/Common"),
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/signage.launch.xml"]),
        ("share/" + package_name + "/config", ["config/signage_param.yaml"]),
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
    description=("signage provides a GUI for passanger."),
    license="TODO",
    entry_points={
        "console_scripts": [
            "signage = signage.signage:main",
        ]
    },
)
