import functools
import logging
from typing import Optional, Union


def log_exceptions(
    reraise: bool = False,
    logger: Optional[Union[str, logging.Logger]] = None,
    level: int = logging.ERROR,
    message: Optional[str] = None
):
    """
    Decorator to catch and log exceptions in a function or method.

    Args:
        reraise (bool): If True, re-raises exceptions after logging. Default: False.
        logger (str or logging.Logger, optional): Logger name or instance. 
            If None, tries to use self.logger if present, else root logger.
        level (int): Logging level to use for the exception message. Default: logging.ERROR.
        message (str, optional): Custom message to log instead of the default exception message.

    Usage:
        @log_exceptions(reraise=True, logger="myLogger", level=logging.DEBUG)
        def some_function(...):
            ...
    """
    def decorator(func):
        """
        Inner decorator function that wraps the target function.

        Args:
            func (Callable): The function to wrap and monitor for exceptions.

        Returns:
            Callable: The wrapped function with exception logging added.
        """
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            """
            Wrapper function that executes the original function, catches exceptions,
            logs them, and optionally re-raises them.

            Returns:
                Any: The return value of the original function, if no exception occurs.
            """
            try:
                return func(*args, **kwargs)
            except Exception as e:
                # Determine which logger to use
                if logger:
                    # Use the provided logger instance or resolve by name
                    log = logger if isinstance(logger, logging.Logger) else logging.getLogger(logger)
                elif args and hasattr(args[0], 'logger'):
                    # Use logger from self.logger if available (for class methods)
                    log = getattr(args[0], 'logger')
                else:
                    # Fallback to root logger
                    log = logging.getLogger()

                # Construct log message
                log_msg = message or f"Exception in {func.__qualname__}: {e}"

                # Log the exception with traceback
                log.log(level, log_msg, exc_info=True)

                # Optionally re-raise the exception
                if reraise:
                    raise

        return wrapper
    return decorator


def log_exceptions_debug(
    reraise: bool = False,
    logger: Optional[Union[str, logging.Logger]] = None,
    message: Optional[str] = None
):
    """
    Shortcut decorator to log exceptions at DEBUG level.

    Args:
        reraise (bool): If True, re-raises exceptions after logging. Default: False.
        logger (str or logging.Logger, optional): Logger name or instance.
        message (str, optional): Custom message to log instead of default exception message.

    Usage:

        @log_exceptions_debug(reraise=True)
        def some_function(...):
            ...
    """
    return log_exceptions(reraise=reraise, logger=logger, level=logging.DEBUG, message=message)
