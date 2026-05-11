"""
Isaac Sim Scene Setup

Functions for building the simulation world:
- World creation with sensible defaults
- Robot loading from USD
- (Future: obstacles, sensors, environment)

Keeps simulation.py's main() function short and readable.
"""
import carb

from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage

from isaac_sim import config


def create_world() -> World:
    """Create the Isaac Sim World with project default settings.

    Returns:
        Initialized World instance with a default ground plane.
    """
    world = World(stage_units_in_meters=config.STAGE_UNITS_IN_METERS)
    world.scene.add_default_ground_plane()
    return world


def resolve_robot_usd_path() -> str:
    """Build the full path to the robot USD on the Isaac Sim asset server.

    Returns:
        Full USD path as a string.

    Raises:
        RuntimeError: if the Isaac Sim asset server cannot be reached.
    """
    assets_root = get_assets_root_path()
    if assets_root is None:
        msg = "Could not find Isaac Sim asset server (nucleus or local cache)"
        carb.log_error(msg)
        raise RuntimeError(msg)
    return assets_root + config.ROBOT_USD_RELATIVE_PATH


def load_robot(world: World) -> Robot:
    """Load the configured robot into the world.

    Imports the USD, registers a Robot wrapper with the scene,
    and returns the Robot handle.

    Args:
        world: The Isaac Sim World to load the robot into.

    Returns:
        The Robot wrapper, ready to be controlled after world.reset().
    """
    usd_path = resolve_robot_usd_path()
    add_reference_to_stage(
        usd_path=usd_path,
        prim_path=config.ROBOT_PRIM_PATH,
    )
    robot = world.scene.add(
        Robot(
            prim_path=config.ROBOT_PRIM_PATH,
            name=config.ROBOT_NAME,
        )
    )
    return robot


def setup_scene() -> tuple:
    """Build the full scene: world, robot, ready for simulation.

    This is the high-level entry point used by main().

    Returns:
        Tuple (world, robot, articulation_controller):
            world: the World instance (call world.step() each frame)
            robot: the Robot wrapper (call robot.get_world_pose() etc)
            articulation_controller: for applying wheel velocity commands

    Note:
        Calls world.reset() internally to initialize physics.
        The articulation controller is only available after reset.
    """
    world = create_world()
    robot = load_robot(world)

    # Initialize physics — must happen before articulation controller is available
    world.reset()

    articulation_controller = robot.get_articulation_controller()
    return world, robot, articulation_controller