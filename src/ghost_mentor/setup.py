from setuptools import setup
import os
from glob import glob

package_name = "ghost_mentor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="AR Ghost Mentor — 암묵지 전수 시스템",
    license="MIT",
    entry_points={
        "console_scripts": [
            "veteran_recorder = ghost_mentor.veteran_recorder:main",
            "ar_overlay_node  = ghost_mentor.ar_overlay_node:main",
            "score_dashboard  = ghost_mentor.score_dashboard:main",
            "save_learner_data = ghost_mentor.save_learner_data:main",
        ],
    },
)
