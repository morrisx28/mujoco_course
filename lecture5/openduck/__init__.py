"""Given Open Duck support code shared by lectures 5 and 6."""
from .reference import (WalkConfig, generate_reference, support_halfspaces,
                        support_margin, diagnostics, print_diagnostics, ROBOT_DIR)
from .motion import validate_episode, save_episode, load_episode, frame_slices

