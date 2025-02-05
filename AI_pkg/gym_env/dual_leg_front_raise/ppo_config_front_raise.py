
"""
This file is to define the PPO configuration for the front raise task.
"""
from dataclasses import dataclass, field
from typing import Optional

from gym_env.ppo_config import PPOconfig

@dataclass
class PPOconfigFrontRaise(PPOconfig):
    """
    Configuration for the front raise task. Inherits from PPOconfig.

    If custom_save_path is not provided, it defaults to:
    ./Model/PPO_dog_front_raise.pt
    """
    custom_save_path: Optional[str] = field(
        default_factory=lambda: "./Model/PPO_dog_front_raise.pt"
    )

    def __post_init__(self):
        """
        Post-initialization hook that updates the SAVE_MODEL_PATH and LOAD_MODEL_PATH
        after the parent class (PPOconfig) attributes are set.
        """
        super().__init__()

        print(f"Custom save path: {self.custom_save_path}")
        self.SAVE_MODEL_PATH = self.custom_save_path
        self.LOAD_MODEL_PATH = self.custom_save_path
