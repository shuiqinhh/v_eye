from setuptools import setup

package_name = "veye_local"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={package_name: "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/offline_mapping.launch.py",
            "launch/online_localization.launch.py",
        ]),
        ("share/" + package_name + "/config", ["config/default.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="veye",
    maintainer_email="user@todo.todo",
    description="Lightweight single-PC V-Eye navigation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "video_player = veye_local.video_player:main",
            "mbl_localizer = veye_local.mbl_localizer:main",
            "integrator = veye_local.integrator:main",
        ],
    },
)
