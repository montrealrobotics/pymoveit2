from typing import Type
from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory


def enum_to_str(enum_class: Type, value: int) -> str:
    """Converts a ROS2 enum value to its string name.

    Args:
        enum_class: The ROS2 message class containing the enum constants.
        value: The integer value of the enum.

    Returns:
        str: The name of the enum constant, or the value with "UNKNOWN NAME" if not found.
    """
    mapping = {}
    # Iterate over all attributes in the enum class
    for attr_name in dir(enum_class):
        # Consider only uppercase attributes (common convention for enums)
        if attr_name.isupper():
            attr_value = getattr(enum_class, attr_name)
            # Check if the attribute is an integer (enum values are typically int)
            if isinstance(attr_value, int):
                mapping[attr_value] = attr_name
    return mapping.get(value, f"{value} :UNKNOWN NAME")

def scale_velocity(trajectory: JointTrajectory, factor: float) -> JointTrajectory:
    """
    Scales the velocities and accelerations in a JointTrajectory by a given factor.
    """
    for point in trajectory.points:
        point.time_from_start = scale_duration(point.time_from_start, factor)

        point.velocities = [v / factor for v in point.velocities]
        point.accelerations = [a / (factor ** 2) for a in point.accelerations]

    return trajectory

def scale_duration(duration_msg: DurationMsg, factor: float) -> DurationMsg:
    """
    Scales a DurationMsg by a given factor.
    """
    total_nanoseconds = duration_msg.sec * 1_000_000_000 + duration_msg.nanosec
    scaled_nanoseconds = int(total_nanoseconds * factor)

    scaled_msg = DurationMsg()
    scaled_msg.sec, scaled_msg.nanosec = divmod(scaled_nanoseconds, 1_000_000_000)

    return scaled_msg