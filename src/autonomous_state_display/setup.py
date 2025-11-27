#!/usr/bin/env python3

import os

from setuptools import setup


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


package_name = "autonomous_state_display"
setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/autonomous_state_display.launch.xml"]),
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
    description=("autonomous_state_display provides a GUI to display autonomous state."),
    license="TODO",
    entry_points={
        "console_scripts": [
            "autonomous_state_display = autonomous_state_display.autonomous_state_display:main",
        ]
    },
)
