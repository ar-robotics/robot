class ControllerException(Exception):
    """Generic exception for the controller module."""


class NotInProductionMode(ControllerException):
    """Raised when a method requires production mode to be enabled.

    Production mode indicates if the expasion board should be used or not.
    """
