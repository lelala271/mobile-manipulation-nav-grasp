from setuptools import find_packages, setup

package_name = "perception_3d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mobile-manipulation-maintainers",
    maintainer_email="noreply@example.com",
    description="RGB-D target localization nodes for mobile manipulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mock_detector = perception_3d.mock_detector:main",
            "depth_projector = perception_3d.depth_projector:main",
        ],
    },
)
