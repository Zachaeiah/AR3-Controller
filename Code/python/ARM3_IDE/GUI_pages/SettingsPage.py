from .GUI_template import PageBase
import tkinter as tk

class SettingsPage(PageBase):
    """_summary_

    Args:
        PageBase (_type_): _description_
    """
    PAGE_NAME = "Settings"
    
    def __init__(self, parent, controller, logger=None):
        """_summary_

        Args:
            parent (_type_): _description_
            controller (_type_): _description_
        """

        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, title=self.PAGE_NAME, logger=logger)

         # Reflect current app debug mode
        self.debug_var = tk.BooleanVar(value=controller.debug_state)

        self.debug_checkbox = tk.Checkbutton(
            self,
            text="Enable Debug Mode",
            variable=self.debug_var,
            command=self.toggle_debug_mode
        )
        self.debug_checkbox.grid(row=0, column=0, pady=10, sticky="w")

    def toggle_debug_mode(self):
        state = self.debug_var.get()
        self.controller.toggle_debug(state)
        self.logger.info(f"Debug checkbox toggled: {state}")

    def on_show(self):
        self.debug_var.set(self.controller.debug_state)
        self.logger.debug(f"Switching to {self.PAGE_NAME}")