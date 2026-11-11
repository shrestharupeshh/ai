from setuptools import setup

package_name = "agrirover_base"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]), ("share/" + package_name, ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True, maintainer="AgriRover maintainers", maintainer_email="maintainer@example.com",
    description="Bridge to the AgriRover base controller", license="MIT",
    entry_points={"console_scripts": ["base_node = agrirover_base.base_node:main"]},
)
