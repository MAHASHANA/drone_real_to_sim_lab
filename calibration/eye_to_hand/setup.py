from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = "drone_handeye_calibration"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Satya",
    maintainer_email="satya@example.com",
    description="SO-101 and D455 eye-to-hand calibration integration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "charuco_tf_publisher = "
            "drone_handeye_calibration.charuco_tf_publisher:main",
            "so101_telemetry_publisher = "
            "drone_handeye_calibration.so101_telemetry_publisher:main",
        ],
    },
)
