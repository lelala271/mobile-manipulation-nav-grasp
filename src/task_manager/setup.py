from setuptools import find_packages, setup

package_name = "task_manager"

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
    description="Task state machine for mobile manipulation demos.",
    license="MIT",
    entry_points={"console_scripts": ["pick_place_state_machine = task_manager.pick_place_state_machine:main"]},
)
