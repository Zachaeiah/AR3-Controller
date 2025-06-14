# GUI_pages/GUI_template.py
import tkinter as tk
from tkinter import ttk
import logging
import logging.config
from .styles import LARGE_FONT
from .styles import SMALL_FONT

class PageBase(tk.Frame):
    """
    Base class for all GUI pages in the robot IDE.
    Provides a consistent layout and interface for subpages to extend.
    """

    def __init__(self, parent: tk.Widget, controller, title: str = "", subtitle: str = None, logger=None):
        """
        Initialize the page.

        Args:
            parent (tk.Widget): Parent container, typically the main window.
            controller (tk.Tk or App): Reference to main app controller for navigation.
            title (str): Optional page title.
            subtitle (str): Optional subtitle.
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

        # Optional: footer or status bar support could go here

    def on_show(self):
        """
        Hook method: called when the page becomes visible.
        Override this in subclasses to refresh data or update state.
        """
        print(f"switching to {self.PAGE_NAME}")