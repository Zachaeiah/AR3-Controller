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

def interpolate_color(t: float) -> str:
    t = max(0.0, min(1.0, t))
    if t < 0.5:
        ratio = t / 0.5
        r = int(ratio * 255)
        g = int(255 - ratio * (255 - 165))
        b = 0
    else:
        ratio = (t - 0.5) / 0.5
        r = 255
        g = int(165 * (1 - ratio))
        b = 0
    return f"#{r:02x}{g:02x}{b:02x}"

def update_entry_color(entry: ttk.Entry, value: float, min_val: float, max_val: float, style_prefix="Auto") -> None:
    midpoint = 0.0
    max_distance = max(abs(midpoint - min_val), abs(max_val - midpoint))
    closeness = abs(value - midpoint)
    t = min(closeness / max_distance, 1.0)
    color = interpolate_color(t)

    style_name = f"{style_prefix}_{id(entry)}.TEntry"
    style = ttk.Style()
    style.configure(style_name, fieldbackground=color)
    entry.configure(style=style_name)

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

    HOME_ANGLES: np.ndarray[float] = np.array([0,0,0,0,0,0])
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
        self.Kinematics = controller.Kinematics
        self.arm_config = controller.arm_config
        self.logger: logging.Logger = logger

        # setup the locale pose state
        self.last_tcp_angles = self.HOME_ANGLES
        self.init_tcp_pose, _, _ = self.Kinematics.FK(self.HOME_ANGLES)
        self.last_pose = self.init_tcp_pose

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

        # Set plot bounds and labels
        ox, oy, oz = self.operating_volume
        self.ax.set_xlim([-ox, ox])
        self.ax.set_ylim([-oy, oy])
        self.ax.set_zlim([-oz, oz])

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

        # Set viewing angle
        self.ax.view_init(elev=45, azim=45)

        # Setup canvas
        self.canvas: FigureCanvasTkAgg = FigureCanvasTkAgg(fig, master=self.plot_frame)
        self.canvas_widget: tk.Canvas = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill='both', expand=True)


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
        self.update_btn.grid(row=1, column=0, columnspan=1, pady=10)

        self.preserve_view = tk.BooleanVar(value=True)  # default to preserve
        self.Preserveview_checkbox = tk.Checkbutton(self.control_frame, text="Preserve View", variable=self.preserve_view)
        self.Preserveview_checkbox.grid(row=1, column=1, columnspan=1, pady=10)

        self.Rad_mode = tk.BooleanVar(value=True)  # default to Degs
        self.Rad_mode_checkbox = tk.Checkbutton(self.control_frame, text="Rad node", variable=self.Rad_mode)
        self.Rad_mode_checkbox.grid(row=1, column=2, columnspan=1, pady=10)


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

        self.labels_left: list[str] = ["X: ", "Y: ", "Z:", "U:", "V:", "W:"]
        for i, label in enumerate(self.labels_left):
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

    def populate_solution(self, mode: str, solution: np.ndarray) -> None:
        """
        Populate the corresponding input fields with the computed solution.

        Args:
            mode (str): "left" or "right" — the side that was *just calculated*.
            solution (np.ndarray): The joint angles or TCP pose to insert.
        """
        # # update the joint angles color on distange to max value
        # for i, label in enumerate(self.labels_right):
        #     entry = self.inputs["right"].get(label)
        #     if entry:
        #         value = float(solution[i])  
        #         min_value = self.arm_config.links[i].motor_params["min_angle"]
        #         max_value = self.arm_config.links[i].motor_params["max_angle"]

        #         if not self.Rad_mode:
        #             value = np.deg2rad(value)
                    
        #         update_entry_color(entry, value, min_value, max_value, style_prefix=mode)
                
        if mode == "left":
            # We just calculated joint angles from IK, so update right inputs (FK)
            labels = self.labels_right
            target_inputs = self.inputs["right"]
        elif mode == "right":
            labels = self.labels_left
            target_inputs = self.inputs["left"]
        else:
            return

        for i, label in enumerate(labels):
            entry = target_inputs.get(label)
            if entry:
                value = float(solution[i])
                entry.delete(0, "end")
                entry.insert(0, f"{value:.4f}")

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

    def update_plot(self) -> None:
        """
        Update the 3D plot with either FK or IK visualization.
        """
        selection = self.selection_var.get()
        logger = self.logger

        if selection not in self.inputs:
            if logger:
                logger.info("No input set selected.")
            return

        try:
            values = [float(entry.get()) for entry in self.inputs[selection].values()]

        except ValueError:
            if logger:
                logger.warning("Invalid input values.")
            return

        if selection == "left":

            if not self.Rad_mode.get():
                values[-3:] = np.deg2rad(values[-3:]) # Convert angles from degrees to radians.

            # set 1: set up the pose
            quaternion = self.Kinematics.euler_to_quaternion(values[-3:])
            pose = np.concatenate((values[:3], quaternion))

            # Step 2: Get both IK solutions
            joint_pose1, joint_pose2 = self.Kinematics.IK(pose)

            # Step 3: Verify joint limits
            (valid1, fail_joint1) = self.Kinematics.verify_Kinematics(joint_pose1, FK_IK=True)
            (valid2, fail_joint2) = self.Kinematics.verify_Kinematics(joint_pose2, FK_IK=True)


            if valid1 and not valid2:
                closest_angles = joint_pose1
            elif valid2 and not valid1:
                closest_angles = joint_pose2
            elif valid1 and valid2:
                diff1 = np.sum(np.abs(self.last_tcp_angles - joint_pose1))
                diff2 = np.sum(np.abs(self.last_tcp_angles - joint_pose2))
                closest_angles = joint_pose1 if diff1 < diff2 else joint_pose2
            else:
                max_joint1 = False
                max_joint2 = False

                if len(fail_joint1) == len(fail_joint2):
                    max_joint1 = True
                    max_joint2 = True
                elif len(fail_joint1) < len(fail_joint2):
                    max_joint1 = True
                else:
                    max_joint2 = True

                if max_joint1:
                    for i in fail_joint1:
                        fail_link = self.arm_config.links[i]
                        min_angle = fail_link.motor_params["min_angle"]
                        max_angle = fail_link.motor_params["max_angle"] 
                        if (joint_pose1[i] <= min_angle):
                            joint_pose1[i] = min_angle
                        else:
                            joint_pose1[i] = max_angle

                if max_joint2:
                    for i in fail_joint2:
                        fail_link = self.arm_config.links[i]
                        min_angle = fail_link.motor_params["min_angle"]
                        max_angle = fail_link.motor_params["max_angle"] 
                        if (joint_pose1[i] <= min_angle):
                            joint_pose1[i] = min_angle
                        else:
                            joint_pose1[i] = max_angle

                if (max_joint1 and max_joint2):
                    diff1 = np.sum(np.abs(self.last_tcp_angles - joint_pose1))
                    diff2 = np.sum(np.abs(self.last_tcp_angles - joint_pose2))
                    closest_angles = joint_pose1 if diff1 < diff2 else joint_pose2

                if self.logger:
                    self.logger.warning(
                        f"Both IK solutions are invalid: Sol1-joints {fail_joint1} Sol12-joints {fail_joint2}"
                    )
            

            # Step 5: FK to update pose and chain
            tcp_pose, transform_stack, transform_chain_raw = self.Kinematics.FK(closest_angles)
            base_transform = np.eye(4)
            transform_chain = [base_transform] + transform_chain_raw

            self.last_tcp_angles = closest_angles

            if not self.Rad_mode.get():
                solution = np.rad2deg(closest_angles) # Convert angles from radians to degrees.
            else:
                solution = closest_angles

            # Populate forward kinematics inputs with IK result
            self.populate_solution(mode="left", solution=solution)
        
        elif selection == "right":
            if not self.Rad_mode.get():
                values = np.deg2rad(values) # Convert angles from degrees to radians.

            tcp_pose, transform_stack, transform_chain_raw = self.Kinematics.FK(values)
            base_transform = np.eye(4)
            transform_chain = [base_transform] + transform_chain_raw

            euler = self.Kinematics.quaternion_to_euler(tcp_pose[3:])

            if not self.Rad_mode.get():
                euler = np.rad2deg(euler)
            
            solution = np.concatenate((tcp_pose[:3], euler))

            self.populate_solution(mode="right", solution=solution)
        else:
            print("Invalid selection")
            return

        # === Preserve view if enabled ===
        preserve = self.preserve_view.get()
        if preserve:
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
            zlim = self.ax.get_zlim()
            elev = self.ax.elev
            azim = self.ax.azim

        self.ax.clear()
        positions = []

        for i, T in enumerate(transform_chain):
            origin = T[:3, 3]
            positions.append(origin)

            axes = T[:3, :3]
            axes = axes / (np.linalg.norm(axes, axis=0, keepdims=True) + 1e-8)
            axes *= 100

            self.ax.scatter(*origin, color='k')
            self.ax.quiver(*origin, *axes[:, 0], length=0.3, color='r')
            self.ax.quiver(*origin, *axes[:, 1], length=0.3, color='g')
            self.ax.quiver(*origin, *axes[:, 2], length=0.3, color='b')

            if i > 0:
                prev = positions[i - 1]
                self.ax.plot(
                    [prev[0], origin[0]],
                    [prev[1], origin[1]],
                    [prev[2], origin[2]],
                    color='gray')

        self.ax.set_title(f"TCP Pose: X={tcp_pose[0]:.3f}, Y={tcp_pose[1]:.3f}, Z={tcp_pose[2]:.3f}")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.scatter(*tcp_pose[:3], c='r', marker='o')
        self.ax.scatter(0, 0, 0, color='orange', s=40)

        # Restore view if toggled on
        if preserve:
            self.ax.set_xlim(xlim)
            self.ax.set_ylim(ylim)
            self.ax.set_zlim(zlim)
            self.ax.view_init(elev=elev, azim=azim)
        else:
            # Otherwise apply default bounding box
            ox, oy, oz = self.operating_volume
            self.ax.set_xlim([-ox, ox])
            self.ax.set_ylim([-oy, oy])
            self.ax.set_zlim([-oz, oz])
            self.ax.view_init(elev=45, azim=45)

        self.canvas.draw()
        self.update_matrices(transform_stack)

    def update_matrices(self, matrix_list: np.ndarray[float, float] = None):
        """
        Updates the displayed matrix labels with new NumPy matrices.
        matrix_list: List of 8 numpy arrays

        Args:
            matrix_list (np.ndarray[float, float]): a list of all the matris to update
        """
        if matrix_list == None:
             matrix_list: np.ndarray[float, float] = [np.random.uniform(-10, 10, (4, 4)) for _ in range(8)]

        for m_index, matrix in enumerate(matrix_list):
            label_grid: list[tk.Label] = self.matrix_labels[m_index]
            for i in range(matrix.shape[0]):
                for j in range(matrix.shape[1]):
                    val: float = matrix[i, j]
                    label: tk.Label = label_grid[i][j]
                    label.config(text=f"{val:3.3f}")
                    
                    
                    # color = self.color_matrix[i][j]
                    # label.config(bg=color)


    def on_show(self):
        self.logger.debug(f"Switching to {self.PAGE_NAME}")