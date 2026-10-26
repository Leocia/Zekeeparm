from glob import glob
from setuptools import find_packages, setup

package_name = "zekeep_bringup"
setup(
    name=package_name, version="0.3.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        *[(f"share/{package_name}/description/{part}", glob(f"description/{part}/*"))
          for part in ("urdf", "meshes", "collision")],
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Zekeep Maintainers", maintainer_email="support@example.com",
    description="Hardware and robot description for supervised data capture.",
    license="Apache-2.0",
)
