"""
Obstacle Setup

Adds static cuboid obstacles to the Isaac Sim scene for testing
the robot's obstacle avoidance behavior.

Obstacles are defined in config.OBSTACLE_DEFINITIONS, so adding
or moving obstacles requires no changes to this file.
"""
import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

from isaac_sim import config


def add_obstacles(world: World) -> None:
    """Add the configured obstacles to the world.

    Each obstacle is a uniform cube placed at a fixed world-frame
    position. They have mass and physics, so the robot can knock
    them around — useful for visual feedback if avoidance fails.

    Args:
        world: The Isaac Sim World to add the obstacles to.
    """
    for name, position, size, color in config.OBSTACLE_DEFINITIONS:
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/{name}",
                name=name,
                position=np.array(position),
                scale=np.array([size, size, size]),
                color=np.array(color),
                mass=1.0,
            )
        )