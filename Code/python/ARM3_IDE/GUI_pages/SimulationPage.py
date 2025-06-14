from .GUI_template import PageBase

class SimulationPage(PageBase):
    """_summary_

    Args:
        PageBase (_type_): _description_
    """
    PAGE_NAME = "Simulation"
    
    def __init__(self, parent, controller, logger=None):
        """_summary_

        Args:
            parent (_type_): _description_
            controller (_type_): _description_
        """

        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, title = self.PAGE_NAME)

    def on_show(self):
        self.logger.debug(f"Switching to {self.PAGE_NAME}")