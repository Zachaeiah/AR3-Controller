from .GUI_template import PageBase
import tkinter as tk
from tkinter import ttk
from typing import Optional, Callable, Any
import logging

from . import HomePage, ProgramEditorPage, SimulationPage, ConsolePage, ConnectionPage, ProjectManagerPage, DiagnosticsPage


class SettingsPage(PageBase):
    """SettingsPage displays settings related to the IDE environment."""

    PAGE_NAME: str = "Settings"
    SEL_BTN_TT_MSGS: str = """Here you can change settings about the IDE"""

    def __init__(self, parent: tk.Widget, controller, logger: Optional[logging.Logger] = None):
        """
        _summary_

        Args:
            parent (Widget): The parent container, typically a frame from the main app.
            controller (App): The main application controller used for accessing shared resources.
            logger (Optional[logging.Logger]): Logger instance for logging debug/info/errors.
        """
        super().__init__(parent, controller, title=self.PAGE_NAME)

        self.debug_var: tk.BooleanVar = tk.BooleanVar(value=controller.debug_state)
        self.column_frames: list[ttk.LabelFrame] = []

         # Make sure the grid in this frame expands columns equally
        for col in range(7):
            self.grid_columnconfigure(col, weight=1)  # Equal weight for each column
        self.grid_rowconfigure(0, weight=1)  # Optional: let row 0 expand if needed

        # Set equal weight to both rows to balance height
        self.grid_rowconfigure(0, minsize=100)
        self.grid_rowconfigure(1, minsize=100)

        self.build_top_row()
        self.build_bottom_row()

    def build_top_row(self):
        """Create the top row with 7 configurable setting frames."""
        for col in range(7):
            frame: ttk.LabelFrame = ttk.LabelFrame(self, text=" ")
            frame.grid(row=0, column=col, padx=5, pady=5, sticky="nsew")
            self.column_frames.append(frame)
            self.build_column(col, frame)

    def build_bottom_row(self):
        """Create a full-width frame in row 1 and place the debug checkbox inside it."""
        bottom_frame: ttk.LabelFrame = ttk.LabelFrame(self, text="Globle settings")
        bottom_frame.grid(row=1, column=0, columnspan=7, sticky="nsew", padx=5, pady=10)

        self.debug_checkbox: tk.Checkbutto = tk.Checkbutton(
            bottom_frame,
            text="Enable Debug Mode",
            variable=self.debug_var,
            command=self.toggle_debug_mode
        )
        self.debug_checkbox.pack(anchor="w")  # Left-align inside the bottom frame
        self.TOOLTIPS[self.debug_checkbox] = "then set, logger level is DEBUG\nWhen cleared, logger level is INFO"

    def build_column(self, index: int, frame: ttk.LabelFrame):
        """Dispatch to the appropriate column-building method."""
        build_methods:list[Callable[[ttk.LabelFrame], None]] = [
            self.build_column_0,
            self.build_column_1,
            self.build_column_2,
            self.build_column_3,
            self.build_column_4,
            self.build_column_5,
            self.build_column_6,
        ]
        if 0 <= index < len(build_methods):
            build_methods[index](frame)

    def build_column_0(self, frame: ttk.LabelFrame) -> None:
        frame.configure(text=HomePage.PAGE_NAME)
        tk.Label(frame, text="Option C").pack()

    def build_column_1(self, frame: ttk.LabelFrame) -> None:
        frame.configure(text=ProgramEditorPage.PAGE_NAME)
        tk.Label(frame, text="Option B").pack()

    def build_column_2(self, frame: ttk.LabelFrame) -> None:
        frame.configure(text=SimulationPage.PAGE_NAME)
        tk.Label(frame, text="Option C").pack()

    def build_column_3(self, frame: ttk.LabelFrame) -> None:
        frame.configure(text=ConsolePage.PAGE_NAME)
        tk.Label(frame, text="Option D").pack()

    def build_column_4(self, frame: ttk.LabelFrame) -> None:
        frame.configure(text=ConnectionPage.PAGE_NAME)
        tk.Label(frame, text="Option E").pack()
        
    def build_column_5(self, frame: ttk.LabelFrame) -> None:
        frame.configure(text=ProjectManagerPage.PAGE_NAME)
        tk.Label(frame, text="Option F").pack()

    def build_column_6(self, frame: ttk.LabelFrame) -> None:
        frame.configure(text=DiagnosticsPage.PAGE_NAME)
        tk.Label(frame, text="Option G").pack()

    def toggle_debug_mode(self) -> None:
        state = self.debug_var.get()
        self.controller.toggle_debug(state)
        self.logger.info(f"Debug checkbox toggled: {state}")

    def on_show(self) -> None:
        self.debug_var.set(self.controller.debug_state)
        self.logger.debug(f"Switching to {self.PAGE_NAME}")
