from .GUI_template import PageBase
import tkinter as tk
from tkinter import ttk
from typing import Optional
import logging
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.ticker import MultipleLocator
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
from utils import log_exceptions, log_exceptions_debug, Tooltip

import numpy as np

class SimulationPage(PageBase):
    """_summary_

    Args:
        PageBase (_type_): _description_
    """
    PAGE_NAME = "Simulation"
    SEL_BTN_TT_MSGS =   """Simulate the programd scriped and anilisy moution paramaters"""
    color_matrix: np.ndarray[str, str] = np.array([
    ["#FFCCCC", "#CCFFCC", "#CCCCFF", "#FFCCCC"], 
    ["#FFCCCC", "#CCFFCC", "#CCCCFF", "#CCFFCC"],
    ["#FFCCCC", "#CCFFCC", "#CCCCFF", "#CCCCFF"],
    ["#FFFFFF", "#FFFFFF", "#FFFFFF", "#FFFFFF"]
])
    def __init__(self, parent: tk.Widget, controller, logger: Optional[logging.Logger] = None):
        """
        _summary_

        Args:
            parent (Widget): The parent container, typically a frame from the main app.
            controller (App): The main application controller used for accessing shared resources.
            logger (Optional[logging.Logger]): Logger instance for logging debug/info/errors.
        """

        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, title=self.PAGE_NAME)
        self.logger: logging.Logger = logger

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        self.inputs: dict[str, dict] = {"left": {}, "right": {}}
        self.selection_var = tk.StringVar(value="left")

        # values that will be set by the Kinematics solver
        self.FK_defult_values = [0,0,0,0,0,0]
        self.IK_defult_values = [0,0,0,0,0,0]
        self.oprating_volume = [500, 500, 500]

        self._build_plot_frame()
        self._build_control_frame()



    def _build_plot_frame(self):
        """Creates the left-side 3D plot area with interactive controls and better tick visibility."""

        self.operating_volume = [500, 500, 500]  # Max XYZ values

        self.plot_frame: ttk.Frame = ttk.Frame(self)
        self.plot_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        fig:Figure = Figure(figsize=(5, 5), dpi=100)
        self.ax: Figure.Axes3D = fig.add_subplot(111, projection='3d')

        # Set axis limits
        self.ax.set_xlim([0, self.operating_volume[0]])
        self.ax.set_ylim([0, self.operating_volume[1]])
        self.ax.set_zlim([0, self.operating_volume[2]])

        # Set ticks
        major_locator: MultipleLocator = MultipleLocator(100)
        minor_locator: MultipleLocator = MultipleLocator(25)

        for axis in [self.ax.xaxis, self.ax.yaxis, self.ax.zaxis]:
            axis.set_major_locator(major_locator)
            axis.set_minor_locator(minor_locator)

        self.ax.tick_params(axis='x', which='minor', length=4, color='gray')
        self.ax.tick_params(axis='y', which='minor', length=4, color='gray')
        self.ax.tick_params(axis='z', which='minor', length=4, color='gray')

        # Optional grid for better spatial understanding
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5)

        # Setup canvas
        self.canvas: FigureCanvasTkAgg = FigureCanvasTkAgg(fig, master=self.plot_frame)
        self.canvas_widget: tk.Canvas = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill='both', expand=True)

    def update_plot(self):
        """_summary_
        """
        selection: str = self.selection_var.get()
        if selection not in self.inputs:
            if self.logger:
                self.logger.info("No input set selected.")
            return

        try:
            values: list[float] = [float(entry.get()) for entry in self.inputs[selection].values()]
        except ValueError:
            if self.logger:
                self.logger.warning("Invalid input values.")
            return

        self.ax.clear()
        self.ax.set_title(f"Point at a={values[3]}, {values[4]}, {values[5]}")
        self.ax.set_xlim([-self.operating_volume[0], self.operating_volume[0]])
        self.ax.set_ylim([-self.operating_volume[1], self.operating_volume[1]])
        self.ax.set_zlim([-self.operating_volume[2], self.operating_volume[2]])
        self.ax.scatter(values[0], values[1], values[2], c='r', marker='o')
        self.ax.view_init(elev=45, azim=45)
        self.canvas.draw()

        self.update_matrices()


    def _build_control_frame(self):
        """Creates the right-side control area with two columns and a button."""
        self.control_frame: ttk.Frame = ttk.Frame(self)
        self.control_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        self.control_frame.columnconfigure(0, weight=1)
        self.control_frame.columnconfigure(1, weight=1)

        self._build_left_column()
        self._build_right_column()
        self._build_matrix_display()

        self.update_btn: ttk.Button = ttk.Button(self.control_frame, text="Update Plot", command=self.update_plot)
        self.update_btn.grid(row=1, column=0, columnspan=2, pady=10)


    def _build_left_column(self):
        """Builds the left control column for Set A."""
        left_col_frame: ttk.LabelFrame = ttk.LabelFrame(self.control_frame, text="Set A")
        left_col_frame.grid(row=0, column=0, sticky="nsew", padx=5)

        self.left_use_check: ttk.Checkbutton = ttk.Checkbutton(
            left_col_frame,
            text="Inverse Kinematics",
            variable=self.selection_var,
            onvalue="left",
            offvalue="",
            command=self.sync_checkboxes
        )
        self.left_use_check.grid(row=0, column=0, columnspan=2, pady=(0, 5))

        labels_left: list[str] = ["X: ", "Y: ", "Z:", "U:", "V:", "W:"]
        for i, label in enumerate(labels_left):
            ttk.Label(left_col_frame, text=label).grid(row=i+1, column=0, sticky="e")
            entry: ttk.Entry = ttk.Entry(left_col_frame)
            entry.insert(0, str(self.IK_defult_values[i]))
            entry.grid(row=i+1, column=1)
            self.inputs["left"][label] = entry


    def _build_right_column(self):
        """Builds the right control column for Set B."""
        right_col_frame = ttk.LabelFrame(self.control_frame, text="Set B")
        right_col_frame.grid(row=0, column=1, sticky="nsew", padx=5)

        self.right_use_check: ttk.Checkbutton = ttk.Checkbutton(
            right_col_frame,
            text="Forward Kinematics",
            variable=self.selection_var,
            onvalue="right",
            offvalue="",
            command=self.sync_checkboxes
        )
        self.right_use_check.grid(row=0, column=0, columnspan=2, pady=(0, 5))

        self.labels_right: list[str] = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        for i, label in enumerate(self.labels_right):
            ttk.Label(right_col_frame, text=label).grid(row=i+1, column=0, sticky="e")
            entry:ttk.Entry = ttk.Entry(right_col_frame)
            entry.insert(0, str(self.FK_defult_values[i]))
            entry.grid(row=i+1, column=1)
            self.inputs["right"][label] = entry
    
    def sync_checkboxes(self):
        """_summary_
        """
        # Prevent deselecting both
        current: str = self.selection_var.get()
        if current not in ("left", "right"):
            # Revert to previous state (default to left)
            self.selection_var.set("left")

    def _build_matrix_display(self):
        """Creates 8 labeled matrix displays below the control frame."""

        self.matrix_frame: ttk.Frame = ttk.Frame(self.control_frame)
        self.matrix_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(10, 0))

        # This will hold references to the label widgets so we can update them later
        self.matrix_labels: list[list[tk.Label]] = []
        self.matrix_names: list[str] = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6", "Tool Frame", "Wild Card"]

        for m_index in range(8):
            matrix_label_frame: ttk.LabelFrame = ttk.LabelFrame(self.matrix_frame, text=self.matrix_names[m_index])
            matrix_label_frame.grid(row=m_index // 2, column=m_index % 2, padx=5, pady=5, sticky="nsew")

            matrix: np.ndarray[float, float] = np.zeros((4, 4))  # Default shape: 3x3, you can change this

            label_grid = []
            for i in range(matrix.shape[0]):
                row_labels = []
                for j in range(matrix.shape[1]):
                    val: float = matrix[i, j]
                    lbl: tk.Label = tk.Label(matrix_label_frame, text=f"{val:.3f}", width=6, relief="ridge", anchor="center")
                    lbl.grid(row=i, column=j, padx=1, pady=1)
                    color: str = self.color_matrix[i][j]
                    lbl.config(bg=color)
                    row_labels.append(lbl)
                label_grid.append(row_labels)

            self.matrix_labels.append(label_grid)

    def update_matrices(self):
        """
        Updates the displayed matrix labels with new NumPy matrices.
        matrix_list: List of 8 numpy arrays
        """
        matrix_list: np.ndarray[float, float] = [np.random.uniform(-10, 10, (4, 4)) for _ in range(8)]

        for m_index, matrix in enumerate(matrix_list):
            label_grid: list[tk.Label] = self.matrix_labels[m_index]
            for i in range(matrix.shape[0]):
                for j in range(matrix.shape[1]):
                    val: float = matrix[i, j]
                    label: tk.Label = label_grid[i][j]
                    label.config(text=f"{val:.3f}")
                    
                    
                    # color = self.color_matrix[i][j]
                    # label.config(bg=color)


    def on_show(self):
        self.logger.debug(f"Switching to {self.PAGE_NAME}")