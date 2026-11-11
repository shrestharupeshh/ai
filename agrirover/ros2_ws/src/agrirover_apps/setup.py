from setuptools import setup

package_name = "agrirover_apps"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]), ("share/" + package_name, ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True, maintainer="AgriRover maintainers", maintainer_email="maintainer@example.com",
    description="AgriRover mission, inspection and sensing nodes", license="MIT",
    entry_points={"console_scripts": ["mission_node = agrirover_apps.mission_node:main", "env_sensor_node = agrirover_apps.env_sensor_node:main"]},
)
