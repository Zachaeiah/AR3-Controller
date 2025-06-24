import tkinter as tk
import os
from tkinter import ttk
from GUI_pages import *
from PIL import Image, ImageTk
from robot import RobotArmConfig, RobotKinematics
from utils import log_exceptions, log_exceptions_debug, Tooltip
import logging
import logging.config
from typing import *
from typing import Union

import numpy as np



class App(tk.Tk):
    """_summary_

    Args:
        tk (_type_): _description_
    """
    config_path = "data/Configs/robot_arm_config.json"
    pages = ALL_PROGRAM_PAGES
    TOOLTIPS: dict[tk.Widget, str] = {}
    AssetsPaths: dict = {"folder": "assets/icons8-folder-50.png",
                         "file": "assets/icons8-file-26.png"}


    def __init__(self, *args, **kwargs):
        """_summary_
        """
        super().__init__(*args, **kwargs)
        ttk.Style().theme_use('clam')
    

        self.logger: logging.Logger = logging.getLogger(__name__)
        self.debug_state: bool = True
        self.logger.setLevel(logging.DEBUG if self.debug_state else logging.INFO)
        self.logger.info(f"Logging level: {self.logger.level}")

        self.Robot_connect_state: bool = False
        self.logger.info(f"Robot connect State: {self.Robot_connect_state}")

        self.arm_config: RobotArmConfig = RobotArmConfig.from_json(self.config_path)
        self.logger.info(f"Loading robot configuration: {self.config_path}")

        self.Kinematics = RobotKinematics(self.arm_config)

        self.icons: dict[str, Image.Image] = {}
        self.frames: dict[PageBase, PageBase] = {}
        self.nav_buttons: dict[PageBase, tk.Button] = {}

        self.init_window()
        self.load_icon_assets()
        self.setup_layout()
        self.populate_tree(self.base_data_path)

        self.create_pages()
        self.create_menu_bar()
        self.create_toolbar()

        self.switch_page(SimulationPage)

        self.apply_tooltips()

    @log_exceptions_debug()
    def init_window(self):
        self.title("Robot Arm IDE")
        self.geometry("1800x1100")

    @log_exceptions()
    def load_icon_assets(self):
        try:
            for key, value in self.AssetsPaths.items():
                try:
                    pil_img = Image.open(value).resize((16, 16))
                    self.icons[key] = ImageTk.PhotoImage(pil_img)
                except Exception as e:
                    self.logger.warning("Failed to load icon '%s': %s", key, e, exc_info=True)
                    self.icons[key] = None
        except Exception as e:
            self.logger.error("Unexpected error while loading icons: %s", e, exc_info=True)

        self.logger.info("Icon assets loaded")

    @log_exceptions_debug()
    def setup_layout(self):
        # Toolbar
        self.toolbar:tk.Frame = tk.Frame(self, bg="#ddd", height=40)
        self.toolbar.pack(side="top", fill="x")

        # Main layout
        self.main_layout:tk.Frame = tk.Frame(self)
        self.main_layout.pack(side="top", fill="both", expand=True)

        # Treeview Panel (Left)
        self.tree_frame:tk.Frame = tk.Frame(self.main_layout, width=250, bg="white")
        self.tree_frame.pack(side="left", fill="y")

        self.tree:ttk.Treeview = ttk.Treeview(self.tree_frame, columns=("fullpath",), show="tree")
        self.tree.column("fullpath", width=0, stretch=False)
        self.tree.pack(side="left", fill="both", expand=True)

        self.scrollbar: ttk.Scrollbar = ttk.Scrollbar(self.tree_frame, orient="vertical", command=self.tree.yview)
        self.scrollbar.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=self.scrollbar.set)
        self.tree.heading("#0", text="Directory", anchor="w")

        self.base_data_path:str = os.path.abspath('data')
        self.tree.bind('<<TreeviewOpen>>', self.on_treeview_open)
        self.tree.bind('<Double-1>', self.on_treeview_double_click)
        self.TOOLTIPS[self.tree] = "all available files in the IDE"

        # Page Container (Right)
        self.container: tk.Frame = tk.Frame(self.main_layout)
        self.container.pack(side="left", fill="both", expand=True)
        self.container.grid_rowconfigure(0, weight=1)
        self.container.grid_columnconfigure(0, weight=1)

        self.logger.debug(f"Initializing UI Setup for {self.__class__.__name__}")

    def create_pages(self) -> None:
        """Create and initialize each page in the application."""
        for PageClass in self.pages:
            try:
                page: PageBase = PageClass(self.container, self, logger=self.logger)
                self.frames[PageClass] = page
                page.grid(row=0, column=0, sticky="nsew")
                page.apply_tooltips() # apply all tooltips when the pahe is crated 
            except Exception as e:
                self.logger.error(f"Failed to create page {PageClass.__name__}: {e}", exc_info=True)
           
    @log_exceptions()
    def create_menu_bar(self) -> None:
        """_summary_
        """
        menu_bar = tk.Menu(self)

        file_menu: tk.Menu = tk.Menu(menu_bar, tearoff=0)
        file_menu.add_command(label="New")
        file_menu.add_command(label="Open")
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.quit)
        menu_bar.add_cascade(label="File", menu=file_menu)

        settings_menu: tk.Menu = tk.Menu(menu_bar, tearoff=0)
        settings_menu.add_command(label="Editor Settings", command=self.open_editor_settings)

        theme_submenu: tk.Menu = tk.Menu(settings_menu, tearoff=0)
        theme_submenu.add_command(label="Light", command=lambda: self.set_theme("light"))
        theme_submenu.add_command(label="Dark", command=lambda: self.set_theme("dark"))
        settings_menu.add_cascade(label="Theme", menu=theme_submenu)

        connection_submenu:tk.Menu = tk.Menu(settings_menu, tearoff=0)
        connection_submenu.add_command(label="Connect", command=self.connect_robot)
        connection_submenu.add_command(label="Disconnect", command=self.disconnect_robot)
        settings_menu.add_cascade(label="Connection", menu=connection_submenu)

        menu_bar.add_cascade(label="Settings", menu=settings_menu)

        help_menu: tk.Menu = tk.Menu(menu_bar, tearoff=0)
        help_menu.add_command(label="About", command=self.show_about)
        menu_bar.add_cascade(label="Help", menu=help_menu)

        self.config(menu=menu_bar)

##### testing methids to set menu commed var 
    def open_editor_settings(self):
        print("Opening editor settings...")

    def set_theme(self, theme_name):
        print(f"Switching to {theme_name} theme...")

    def connect_robot(self):
        print("Connecting to robot...")

    def disconnect_robot(self):
        print("Disconnecting robot...")

    def show_about(self):
        tk.messagebox.showinfo("About", "Robot Arm IDE\nInspired by ABB RAPID")
##### testing methids to set menu commed var 
    @log_exceptions()
    def create_toolbar(self) -> None:
        """
        Create navigation toolbar buttons for each page class in self.pages.
        """

        # Left and right containers inside the toolbar
        left_frame: tk.Frame = tk.Frame(self.toolbar, bg="#ddd")
        right_frame: tk.Frame = tk.Frame(self.toolbar, bg="#ddd")

        left_frame.pack(side="left", fill="x", expand=True)
        right_frame.pack(side="right")

        # Create navigation buttons (left side)
        for PageClass in self.pages:
            page_name: str = getattr(PageClass, "PAGE_NAME", PageClass.__name__)

            btn:tk.Button = tk.Button(
                left_frame,
                text=page_name,
                command=lambda p=PageClass: self.switch_page(p),
                relief=tk.FLAT,
                padx=10,
                pady=5,
                bg="#eee"
            )
            btn.pack(side="left", padx=2, pady=5)
            self.nav_buttons[PageClass] = btn

            if hasattr(PageClass, "SEL_BTN_TT_MSGS"):
                TT_msg: str = getattr(PageClass, "SEL_BTN_TT_MSGS")
                self.TOOLTIPS[self.nav_buttons[PageClass]] = TT_msg


        # Create connect button (right side)
        self.connect_button: tk.Button = tk.Button(
            right_frame,
            text="Connect",
            command=self.handle_connect,
            relief=tk.RAISED,
            padx=10,
            pady=5,
            bg="red"
        )
        self.connect_button.pack(side="right", padx=10, pady=5)
        self.TOOLTIPS[self.connect_button] = "Connect to the robot controller.\nUseful for diagnostics or control."
        
    @log_exceptions()
    def switch_page(self, page_class: Type[PageBase]) -> None:
        """
        Switch the visible frame to the selected page.
        Highlights the active button and calls the on_show() method if present.

        Args:
            page_class (_type_): The page class to switch to.
        """
        
        frame: PageBase = self.frames[page_class]
        frame.tkraise()

        if hasattr(frame, "on_show"):
            try:
                frame.on_show()
            except Exception as e:
                self.logger.warning(f"{page_class.__name__}.on_show() raised an error: {e}", exc_info=True)

        # Update toolbar button appearances
        for cls, btn in self.nav_buttons.items():
            if cls == page_class:
                btn.config(bg="#bbb", relief=tk.SUNKEN)
            else:
                btn.config(bg="#eee", relief=tk.FLAT)

    def populate_tree(self, path : str, parent : str='') -> None:
        """
        Populate the tree view with directories and files starting from a given path.

        Args:
            path (str): The starting file system path.
            parent (str): Treeview parent node ID. Defaults to '' (root).
        """
        try:
            items: list[str] = os.listdir(path)
        except PermissionError as e:
            self.logger.warning(f"Permission denied while accessing: {path}")
            self.logger.debug("PermissionError stack trace:", exc_info=True)
            return
        except FileNotFoundError as e:
            self.logger.warning(f"Path not found: {path}")
            self.logger.debug("FileNotFoundError stack trace:", exc_info=True)
            return
        except Exception as e:
            self.logger.error(f"Unexpected error reading directory: {path} — {e}")
            self.logger.debug("Unexpected exception stack trace:", exc_info=True)
            return

        for item in sorted(items):
            try:
                full_path: str = os.path.join(path, item)
                is_dir: bool = os.path.isdir(full_path)
                icon: Image.Image = self.icons["folder"] if is_dir else self.icons["file"]

                node_id: str = self.tree.insert(parent, 'end', text=item, image=icon, open=False)
                self.tree.set(node_id, "fullpath", full_path)

                # Preload a dummy child if the directory is not empty (expandable)
                if is_dir and self.has_subitems(full_path):
                    self.tree.insert(node_id, 'end', text='dummy', values=('dummy',))

            except Exception as e:
                self.logger.warning(f"Failed to add item to tree: {item} — {e}")
                self.logger.debug("Item insert exception stack trace:", exc_info=True)

    def has_subitems(self, path: str) -> Union[list[str], bool]:
        """
        Check whether the given directory has subitems.

        Args:
            path (str): Filesystem path to check.

        Returns:
            list[str]: List of item names if directory has items.
            bool: False if the directory is empty or cannot be read.
        """
        try:
            items = os.listdir(path)
            return items if items else False
        except Exception as e:
            self.logger.error(f"Unexpected error reading directory: {path} — {e}")
            self.logger.debug("Unexpected exception stack trace:", exc_info=True)
            return False

    @log_exceptions()
    def on_treeview_open(self, event: tk.Event) -> None:
        """_summary_

        Args:
            event (_type_): _description_
        """
        node: str = self.tree.focus()
        path: Any = self.tree.set(node, "fullpath")
        children: tuple[str, ...] = self.tree.get_children(node)

        dummy_found: bool = any(self.tree.item(child, 'text') == 'dummy' for child in children)
        if dummy_found:
            self.tree.delete(*children)
            self.populate_tree(path, node)

    @log_exceptions()
    def on_treeview_double_click(self, event: tk.Event) -> None:
        """
        Handle double-click events on the Treeview widget.
        If a file node is double-clicked, log its relative path.

        Args:
            event (tkinter.Event): The event object containing metadata about the double-click event.
        """
        item: str = self.tree.focus()
        path: str = self.get_full_path(item)
        
        if os.path.isfile(path):
            rel_path: str = os.path.relpath(path, self.base_data_path)
            self.logger.debug(f"[OPEN FILE] {rel_path}")

    @log_exceptions()
    def get_full_path(self, node: str) -> str:
        """
        Retrieve the full file path associated with a tree node.

        Args:
            node (str): The identifier of the tree node (usually returned by tree.insert).

        Returns:
            str: The full file path stored in the 'fullpath' field of the node.
                If the node is invalid or does not have a path, returns the default 'data' directory path.
        """
        return self.tree.set(node, "fullpath") if self.tree.set(node) else os.path.abspath('data')

    @log_exceptions()
    def toggle_debug(self, enable: bool)  -> None:
        self.debug_state = enable
        new_level:int = logging.DEBUG if enable else logging.INFO
        self.logger.setLevel(new_level)
        self.logger.info(f"Debug mode set to {enable}")

    @log_exceptions()
    def handle_connect(self) -> None:
        """
        This will connect the robot ot the IDE
        """
        self.logger.debug("Connect button clicked")
        # Simulate connection toggle
        if self.Robot_connect_state:
            self.connect_button.config(text="Disconnected", bg="red")
            self.Robot_connect_state = False
        else:
            self.connect_button.config(text="Connected", bg="green")
            self.Robot_connect_state = True

    @log_exceptions()
    def apply_tooltips(self) -> None:
        """
        will apply Tool tips for all  tk.Widget in TOOLTIPS dict
        """
        for key, value in self.TOOLTIPS.items():
            Tooltip(key, value)

def jsondemo():
    config_path = "data/Configs/robot_arm_config.json"

    # --- Load Configuration ---
    print("Loading robot configuration...")
    arm_config = RobotArmConfig.from_json(config_path)

    # --- Print Motor Parameters Before ---
    print("\n--- BEFORE ---")
    for link in arm_config.links:
        print(f"{link.name}: {link}")

    # --- Modify Motor Parameters ---
    print("\nModifying motor parameters...")
    arm_config.update_link(
        key="J1",  # Use ID or name
        motor_params={
            "max_velocity": 500,
            "max_acceleration": 700
        }
    )
    arm_config.update_link(
        key="J5",
        DenaHar_params={
            "alpha": 1.57079633
        }
    )

    # --- Save the Modified Configuration ---
    arm_config.to_json(config_path)
    print("\nConfiguration updated and saved.")

    # --- Reload and Print Motor Parameters After ---
    print("\n--- AFTER ---")
    updated_config = RobotArmConfig.from_json(config_path)
    for link in updated_config.links:
        print(f"{link.name}:\n{link}")

if __name__ == "__main__":

    LOG_FILE_PATH = os.path.join("data", "logs", "Robot_IDE_long.log")
    os.makedirs(os.path.dirname(LOG_FILE_PATH), exist_ok=True)
    logging.config.fileConfig('data/Configs/logger_config.conf')

    config_path = "data/Configs/robot_arm_config.json"
    arm_config = RobotArmConfig.from_json(config_path)

    #init pose
    # angles = np.radians([0, 0, 0, 0, 0, 0])

    # kin = RobotKinematics(arm_config)
    # tcp_pose, _, _ = kin.FK(angles)

    # # tcp_pose
    # joint_pose1, joint_pose2 = kin.IK(tcp_pose)

    # print(f"imput angles: {np.round(angles, 3)}")
    # print()
    # print(f"FK sol: {np.around(tcp_pose, 3)}")
    # print(f"verify FK tcp_pose:{kin.verify_Kinematics(tcp_pose, FK_IK = False)}")
    # print()
    # print(f"IK sol1: {np.round(joint_pose1, 3)}")
    # print(f"IK sol2: {np.round(joint_pose2, 3)}")
    # print()
    # print(f"verify IK solution 1:{kin.verify_Kinematics(joint_pose1)[0]}")
    # print(f"verify IK solution 2:{kin.verify_Kinematics(joint_pose2)[0]}")
    # print()


    app = App()
    app.mainloop()
    


