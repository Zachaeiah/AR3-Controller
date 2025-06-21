# GUI_pages/GUI_template.py
import tkinter as tk
from tkinter import ttk
from typing import Optional
import logging
from utils import Tooltip

from .styles import LARGE_FONT
from .styles import SMALL_FONT

class PageBase(tk.Frame):
    """
    Base class for all GUI pages in the robot IDE.
    Provides a consistent layout and interface for subpages to extend.
    """
    SEL_BTN_TT_MSGS =   """"""
    TOOLTIPS: dict = {}


    def __init__(self, parent: tk.Widget, controller, title: str = "", subtitle: str = None, logger: Optional[logging.Logger] = None):
        """
        Initialize the page.

        Args:
            parent (Widget): The parent container, typically a frame from the main app.
            controller (App): The main application controller used for accessing shared resources.
            title (str): Optional page title.
            subtitle (str): Optional subtitle.
            logger (Optional[logging.Logger]): Logger instance for logging debug/info/errors.
        """
        super().__init__(parent)
        self.controller = controller
        self.logger = controller.logger

        # Layout configuration
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Title Section
        if title:
            title_label = tk.Label(self, text=title, font=LARGE_FONT)
            title_label.grid(row=0, column=0, sticky="w", padx=10, pady=(10, 0))

        if subtitle:
            subtitle_label = tk.Label(self, text=subtitle, font=SMALL_FONT, fg="gray")
            subtitle_label.grid(row=0, column=0, sticky="e", padx=10, pady=(10, 0))

        # Content Area (child pages place widgets here)
        self.content_frame = ttk.Frame(self)
        self.content_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)

    def apply_tooltips(self) -> None:
        """
        will apply Tool tips for all  tk.Widget in TOOLTIPS dict
        """
        for key, value in self.TOOLTIPS.items():
            Tooltip(key, value)

    def on_show(self) -> None:
        """
        Hook method: called when the page becomes visible.
        Override this in subclasses to refresh data or update state.
        """
        print(f"switching to {self.PAGE_NAME}")