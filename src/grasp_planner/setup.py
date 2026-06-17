from setuptools import find_packages, setup

package_name = "grasp_planner"

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
    description="Simple grasp pose generation for mobile manipulation.",
    license="MIT",
    entry_points={"console_scripts": ["grasp_pose_planner = grasp_planner.grasp_pose_planner:main"]},
)
