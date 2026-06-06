"""
Perception — Obstacle Detection from LaserScan

Pure functions for analyzing LiDAR scan data.
No rclpy dependencies — operates on plain Python lists/sequences.

The scan layout (from sensor_msgs/LaserScan):
    - ranges: list of distance measurements
    - ranges[0] corresponds to angle_min (typically -pi)
    - ranges[i] corresponds to angle_min + i * angle_increment
    - ranges[N/2] corresponds to angle 0 (directly in front)
    - inf or nan values indicate "no return" (out of range or no obstacle)

For obstacle avoidance we typically look at a "front cone" — a slice
of indices centered on the forward-facing direction.
"""
import math


def _front_index_range(
    scan_size: int,
    angle_min: float,
    angle_increment: float,
    half_fov_rad: float,
) -> tuple:
    """Compute the [start, end] index range for the front-facing cone.

    Args:
        scan_size: total number of beams in the scan
        angle_min: angle of the first beam (typically -pi)
        angle_increment: angle between consecutive beams (rad)
        half_fov_rad: half-width of the front cone we care about (rad)

    Returns:
        (start_index, end_index) — slice indices into the ranges array.
    """
    # Index where angle = 0 (directly in front)
    center_index = int(round((0.0 - angle_min) / angle_increment))

    # Number of beams covered by half_fov on each side
    half_window = int(round(half_fov_rad / angle_increment))

    start = max(0, center_index - half_window)
    end = min(scan_size, center_index + half_window + 1)
    return start, end


def distance_to_nearest_obstacle_in_front(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    half_fov_rad: float,
) -> float:
    """Return the closest detected distance within the forward cone.

    Filters out invalid readings (inf, nan, below range_min, above range_max).
    Returns range_max if nothing valid is detected in the cone.

    Args:
        ranges: sequence of range measurements (from LaserScan.ranges)
        angle_min: from LaserScan.angle_min
        angle_increment: from LaserScan.angle_increment
        range_min: from LaserScan.range_min (filter out anything closer)
        range_max: from LaserScan.range_max (use as default if nothing seen)
        half_fov_rad: half-width of the front cone (rad)

    Returns:
        Closest valid range in the cone, in meters.
    """
    start, end = _front_index_range(
        len(ranges), angle_min, angle_increment, half_fov_rad
    )

    closest = range_max
    for r in ranges[start:end]:
        # Skip invalid readings: NaN, inf, out-of-range
        if not math.isfinite(r):
            continue
        if r < range_min or r > range_max:
            continue
        if r < closest:
            closest = r
    return closest


def is_obstacle_too_close(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    half_fov_rad: float,
    danger_distance: float,
) -> bool:
    """Return True if anything in the front cone is closer than danger_distance.

    A convenience wrapper around distance_to_nearest_obstacle_in_front().

    Args:
        ranges, angle_min, angle_increment, range_min, range_max: from LaserScan
        half_fov_rad: half-width of the front cone (rad)
        danger_distance: distance threshold (m); closer than this = "too close"

    Returns:
        True if any beam in the cone reads less than danger_distance.
    """
    closest = distance_to_nearest_obstacle_in_front(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
        half_fov_rad=half_fov_rad,
    )
    return closest < danger_distance
