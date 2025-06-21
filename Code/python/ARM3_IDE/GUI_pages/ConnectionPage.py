from .GUI_template import PageBase
import tkinter as tk
from tkinter import ttk
from typing import Optional
import logging

class ConnectionPage(PageBase):
    """_summary_

    Args:
        PageBase (_type_): _description_
    """
    PAGE_NAME = "Connection"
    SEL_BTN_TT_MSGS =   """Setup connections setting for Robot and GPIO Mapping"""
    
    def __init__(self, parent: tk.Widget, controller, logger: Optional[logging.Logger] = None):
        """
        _summary_

        Args:
            parent (Widget): The parent container, typically a frame from the main app.
            controller (App): The main application controller used for accessing shared resources.
            logger (Optional[logging.Logger]): Logger instance for logging debug/info/errors.
        """

        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, title = self.PAGE_NAME)

    def on_show(self):
        self.logger.debug(f"Switching to {self.PAGE_NAME}")