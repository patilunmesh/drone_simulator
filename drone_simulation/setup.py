import os
from glob import glob

from setuptools import setup

package_name = "drone_simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.py")),
        (os.path.join("share", package_name), glob("config/*.rviz")),
        (os.path.join("share", package_name), glob("stl/*.stl")),
        (os.path.join("share", package_name), glob("srv/*.srv")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hlab",
    maintainer_email="patilunmeshrao@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "Drone_sim = drone_simulation.sim:main",
            "trajectoryGen = drone_simulation.trajectoryGen",
            "quadrocoptertrajectory = drone_simulation.quadrocoptertrajectory",
            "LQR = drone_simulation.LQR",
            "obstacle_detector = drone_simulation.obst_detector:main",
            "obstacle_generator = drone_simulation.obst_publisher_node:main",
            "utils = drone_simulation.utils",
            "nonlinear_dynamics = drone_simulation.nonlinear_dynamics",
            "drone_gym = drone_simulation.sim_service:main",
            "sim_client = drone_simulation.sim_client:main",
            "future_pred = drone_simulation.future_pred",
            "imu_spoof = drone_simulation.imu_spoofing:main",
            "gps_spoof = drone_simulation.gps_spoofing:main"            
        ],
    },
)
