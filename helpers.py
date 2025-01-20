# Created by Carl Slezak
from abc import ABC, abstractmethod
from magicbot.state_machine import AutonomousStateMachine


class Component(ABC):
    """Abstract class for creating components"""

    @abstractmethod
    def execute(self):
        """Required function. Do not call this method from anywhere in the code since the MagicRobot will automatically
        call the execute method of all components at the end of the control loop."""
        pass

    def on_disable(self):
        """Optional function. This function will be called when the robot leaves auton or teleop."""
        pass

    def on_enable(self):
        """Optional function. This function will be called when the robot enters auton or teleop. It should be used
        to initialize the component to a "safe" state so that nothing unexpected happens when the robot is enabled."""
        pass

    def setup(self):
        """Optional function. This function is automatically called after the createObjects function finishes, and
        after all components are created. All variables imported from MagicRobot should be initialized at this point."""
        pass




class Auton(ABC):
    MODE_NAME: str
    DEFAULT: bool
    DISABLED: bool
