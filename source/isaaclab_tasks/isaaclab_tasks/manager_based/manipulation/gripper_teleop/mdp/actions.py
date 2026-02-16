from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.utils import configclass
import isaaclab.envs.mdp as mdp

@configclass
class ActionsCfg:
    """Action specifications for the environment."""
    
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", 
        joint_names=[".*_joint"], 
        scale=1.0, 
        use_default_offset=True
    )
