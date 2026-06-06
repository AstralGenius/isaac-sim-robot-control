"""
Sensor Setup

Creates and configures sensors on the robot, and wires their output
to ROS 2 topics via OmniGraph.

Currently provides:
- setup_lidar(): PhysX-based 2D LiDAR publishing sensor_msgs/LaserScan on /scan
"""
import omni.graph.core as og
import omni.kit.commands
from pxr import Gf

from isaac_sim import config


def setup_lidar() -> str:
    """Attach a PhysX 2D Lidar to the Jetbot and publish /scan via OmniGraph.

    Uses Isaac Sim's PhysX Lidar (the stable, CPU-based sensor system)
    rather than RTX Lidar, because RTX Lidar's IsaacComputeRTXLidarFlatScan
    has a known bug in Isaac Sim 5.0 where it incorrectly reports nonzero
    elevation values for explicitly 2D configurations.

    PhysX Lidar produces identical sensor_msgs/LaserScan output and is
    transparent to downstream ROS 2 consumers.

    Returns:
        The prim path of the created LiDAR sensor.
    """
    lidar_prim_path = config.LIDAR_PRIM_PATH

    # Create the PhysX Lidar sensor.
    # min/max range and angular FOV match the Debug_Rotary profile we used.
    omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path="/lidar",
        parent=config.ROBOT_PRIM_PATH + "/chassis",
        min_range=0.05,
        max_range=30.0,
        draw_points=False,
        draw_lines=True,           # visualize rays in the viewport for debugging
        horizontal_fov=360.0,
        vertical_fov=1.0,          # essentially 0 — strict 2D
        horizontal_resolution=1.0, # 1 degree per beam = 360 beams per scan
        vertical_resolution=1.0,
        rotation_rate=10.0,        # 10 Hz like RPLidar
        high_lod=False,
        yaw_offset=0.0,
        enable_semantics=False,
    )

    # Position the lidar above the chassis
    _set_lidar_translation(lidar_prim_path, config.LIDAR_MOUNT_TRANSLATION)

    # Build the OmniGraph that publishes /scan
    _build_lidar_publish_graph(lidar_prim_path)

    return lidar_prim_path


def _set_lidar_translation(lidar_path: str, translation: tuple) -> None:
    """Position the LiDAR relative to its parent (the chassis)."""
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    lidar_prim = stage.GetPrimAtPath(lidar_path)
    if not lidar_prim.IsValid():
        return
    xform = lidar_prim.GetAttribute("xformOp:translate")
    if xform:
        xform.Set(Gf.Vec3d(*translation))
    else:
        # Create the translate op if missing
        from pxr import UsdGeom
        xformable = UsdGeom.Xformable(lidar_prim)
        xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))


def _build_lidar_publish_graph(lidar_prim_path: str) -> None:
    """Build the OmniGraph that bridges PhysX LiDAR data to ROS 2 /scan.

    The graph runs every simulation tick:
        OnPlaybackTick --> ReadLidarBeams --> PublishLaserScan --> /scan topic

    Args:
        lidar_prim_path: The USD path of the LiDAR sensor prim.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": "/World/LidarROSGraph",
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadLidarBeams", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                ("PublishLaserScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "ReadLidarBeams.inputs:execIn"),
                ("ReadLidarBeams.outputs:execOut", "PublishLaserScan.inputs:execIn"),
                ("ReadLidarBeams.outputs:azimuthRange", "PublishLaserScan.inputs:azimuthRange"),
                ("ReadLidarBeams.outputs:depthRange", "PublishLaserScan.inputs:depthRange"),
                ("ReadLidarBeams.outputs:horizontalFov", "PublishLaserScan.inputs:horizontalFov"),
                ("ReadLidarBeams.outputs:horizontalResolution", "PublishLaserScan.inputs:horizontalResolution"),
                ("ReadLidarBeams.outputs:intensitiesData", "PublishLaserScan.inputs:intensitiesData"),
                ("ReadLidarBeams.outputs:linearDepthData", "PublishLaserScan.inputs:linearDepthData"),
                ("ReadLidarBeams.outputs:numCols", "PublishLaserScan.inputs:numCols"),
                ("ReadLidarBeams.outputs:numRows", "PublishLaserScan.inputs:numRows"),
                ("ReadLidarBeams.outputs:rotationRate", "PublishLaserScan.inputs:rotationRate"),
            ],
            keys.SET_VALUES: [
                ("PublishLaserScan.inputs:topicName", config.TOPIC_SCAN),
                ("PublishLaserScan.inputs:frameId", config.LIDAR_FRAME_ID),
                ("ReadLidarBeams.inputs:lidarPrim", lidar_prim_path),
            ],
        },
    )