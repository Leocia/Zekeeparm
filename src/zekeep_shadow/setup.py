from setuptools import setup

package_name = "zekeep_shadow"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", ["launch/follow.launch.py"]),
        (f"share/{package_name}/config", ["config/servo.yaml"]),
    ],
    install_requires=["setuptools", "pynput"],
    zip_safe=True,
    maintainer="Zekeep Maintainers",
    maintainer_email="support@example.com",
    description="RGB-D red block continuous follower.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "shadow_vision = zekeep_shadow.vision_node:main",
        "web_vision = zekeep_shadow.web_vision:main",
        "shadow_follower = zekeep_shadow.follower_node:main",
    ]},
)
