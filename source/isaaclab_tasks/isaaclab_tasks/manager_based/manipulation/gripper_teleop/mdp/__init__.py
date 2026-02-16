"""MDP definitions for the gripper teleop environment."""

from .observations import *
from .actions import *
from isaaclab.utils import configclass


@configclass
class EventsCfg:
    """Configuration for events."""
    pass


@configclass
class RewardsCfg:
    """Configuration for rewards."""
    pass


@configclass
class TerminationsCfg:
    """Configuration for terminations."""
    pass


@configclass
class CurriculumCfg:
    """Configuration for curriculum."""
    pass
# We will add rewards and others here if needed later
