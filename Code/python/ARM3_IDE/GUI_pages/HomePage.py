from .GUI_template import PageBase
from .styles import SMALL_FONT
import tkinter as tk

class HomePage(PageBase):
    """_summary_

    Args:
        PageBase (_type_): _description_
    """
    PAGE_NAME = "Home"
    
    def __init__(self, parent, controller, logger=None):
        """_summary_

        Args:
            parent (_type_): _description_
            controller (_type_): _description_
        """

        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, title = self.PAGE_NAME)

        title_label = tk.Label(self, text="This page will be used for loading\nrobot project and geting evything setup", font=SMALL_FONT)
        title_label.grid(row=2, column=0, sticky="w", padx=10, pady=(10, 0))
        

    def on_show(self):
        self.logger.debug(f"Switching to {self.PAGE_NAME}")