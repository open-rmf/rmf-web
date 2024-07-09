class AlreadyExistsError(Exception):
    """
    Raised when an a resource already exists and there is a conflict.
    """


class NotFoundError(Exception):
    """
    Raised when a resource is not found.
    """


class InvalidInputError(Exception):
    """
    Raised when an input is invalid.
    """
