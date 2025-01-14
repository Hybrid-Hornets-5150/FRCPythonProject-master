# Created by Carl Slezak
from abc import ABC, abstractmethod
from magicbot.state_machine import AutonomousStateMachine


class Component(ABC):
    """Abstract class for creating components"""

    def __init__(self):
        """Can be overridden if super().__init__() is called."""
        self.enabled = False

    @property
    def ready(self) -> bool:
        """Property of the component. Added for ease of use."""
        return self.is_ready()

    @abstractmethod
    def is_ready(self) -> bool:
        """Required method. Decide if the component is in its ready state (ex: shooter is at speed)."""
        pass

    @abstractmethod
    def run(self):
        """Required method. Implement code to run the component."""
        pass

    @abstractmethod
    def stop(self):
        """Required method. Implement code to stop the component."""
        pass

    def enable(self) -> None:
        """Concrete method that can be overridden if super().enable() is called."""
        self.enabled = True

    def execute(self) -> None:
        """Can be overridden if necessary. Called at the end of the control loop."""
        if self.enabled:
            self.run()
        else:
            self.stop()


class Auton(ABC):
    MODE_NAME: str
    DEFAULT: bool
    DISABLED: bool
