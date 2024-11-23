"""
Keyboard configuration
"""
import dataclasses

@dataclasses.dataclass
class KeyboardConfig:
    """
    Keyboard configuration
    """
    # Keyboard keys
    JOINT_INIT_POS = [0.0, 25.0, 60.0, 0.0, 60.0, 90.0]
    # initial joint position
    KEY_ACTION_MAPPING = {
        'q': (0, 5.0),
        'a': (0, -5.0),
        'w': (1, 5.0),
        's': (1, -5.0),
        'e': (2, 5.0),
        'd': (2, -5.0),
        'r': (3, 5.0),
        'f': (3, -5.0),
        't': (4, 5.0),
        'g': (4, -5.0),
        'y': (5, 5.0),
        'h': (5, -5.0)
    }
    