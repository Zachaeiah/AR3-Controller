import tkinter as tk
import os
from tkinter import ttk
from GUI_pages import *
from PIL import Image, ImageTk
from robot import RobotArmConfig
import logging
import logging.config


class App(tk.Tk):

    pages = ALL_PROGRAM_PAGES
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Load logging config
        logging.config.fileConfig('data/Configs/logger_config.conf')
        self.logger = logging.getLogger(__name__)

        self.debug_state = False
        self.logger.setLevel(logging.INFO)

        self.title("Robot Arm IDE")
        self.geometry("1200x700")

        # Load icons
        file_img = Image.open("assets/icons8-file-26.png").resize((16, 16))
        folder_img = Image.open("assets/icons8-folder-50.png").resize((16, 16))

        self.icons = {
            "folder": ImageTk.PhotoImage(folder_img),
            "file": ImageTk.PhotoImage(file_img)
        }

        # Top toolbar
        self.toolbar = tk.Frame(self, bg="#ddd", height=40)
        self.toolbar.pack(side="top", fill="x")

        # Main layout (treeview + pages)
        self.main_layout = tk.Frame(self)
        self.main_layout.pack(side="top", fill="both", expand=True)

        # Left: Treeview panel
        self.tree_frame = tk.Frame(self.main_layout, width=250, bg="white")
        self.tree_frame.pack(side="left", fill="y")

        self.tree = ttk.Treeview(self.tree_frame, columns=("fullpath",), show="tree")
        self.tree.column("fullpath", width=0, stretch=False)  # Hides the 'fullpath' column
        self.tree.pack(side="left", fill="both", expand=True)

        self.scrollbar = ttk.Scrollbar(self.tree_frame, orient="vertical", command=self.tree.yview)
        self.scrollbar.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=self.scrollbar.set)
        self.tree.heading("#0", text="Directory", anchor="w")

        self.base_data_path = os.path.abspath('data')
        self.populate_tree(self.base_data_path)
        self.tree.bind('<<TreeviewOpen>>', self.on_treeview_open)
        self.tree.bind('<Double-1>', self.on_treeview_double_click)

        # Right: Pages container
        self.container = tk.Frame(self.main_layout)
        self.container.pack(side="left", fill="both", expand=True)

        self.container.grid_rowconfigure(0, weight=1)
        self.container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        self.nav_buttons = {}

        self.create_pages()
        self.create_menu_bar()
        self.create_toolbar()
        self.switch_page(HomePage)

    def create_pages(self):
        for PageClass in self.pages:
            page = PageClass(self.container, self, logger=self.logger)
            self.frames[PageClass] = page
            page.grid(row=0, column=0, sticky="nsew")

    def create_menu_bar(self):
        menu_bar = tk.Menu(self)

        file_menu = tk.Menu(menu_bar, tearoff=0)
        file_menu.add_command(label="New")
        file_menu.add_command(label="Open")
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.quit)
        menu_bar.add_cascade(label="File", menu=file_menu)

        settings_menu = tk.Menu(menu_bar, tearoff=0)
        settings_menu.add_command(label="Editor Settings", command=self.open_editor_settings)

        theme_submenu = tk.Menu(settings_menu, tearoff=0)
        theme_submenu.add_command(label="Light", command=lambda: self.set_theme("light"))
        theme_submenu.add_command(label="Dark", command=lambda: self.set_theme("dark"))
        settings_menu.add_cascade(label="Theme", menu=theme_submenu)

        connection_submenu = tk.Menu(settings_menu, tearoff=0)
        connection_submenu.add_command(label="Connect", command=self.connect_robot)
        connection_submenu.add_command(label="Disconnect", command=self.disconnect_robot)
        settings_menu.add_cascade(label="Connection", menu=connection_submenu)

        menu_bar.add_cascade(label="Settings", menu=settings_menu)

        help_menu = tk.Menu(menu_bar, tearoff=0)
        help_menu.add_command(label="About", command=self.show_about)
        menu_bar.add_cascade(label="Help", menu=help_menu)

        self.config(menu=menu_bar)

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

    def create_toolbar(self):
        """_summary_
        """
        self.nav_buttons = {}
        for PageClass in self.pages:
            page_name = getattr(PageClass, "PAGE_NAME", PageClass.__name__)
            btn = tk.Button(
                self.toolbar,
                text=page_name,
                command=lambda p=PageClass: self.switch_page(p),
                relief=tk.FLAT,
                padx=10,
                pady=5,
                bg="#eee"
            )
            btn.pack(side="left", padx=2, pady=5)
            self.nav_buttons[PageClass] = btn

    def switch_page(self, page_class):
        """_summary_

        Args:
            page_class (_type_): _description_
        """
        frame = self.frames[page_class]
        frame.tkraise()
        if hasattr(frame, "on_show"):
            frame.on_show()
        for cls, btn in self.nav_buttons.items():
            if cls == page_class:
                btn.config(bg="#bbb", relief=tk.SUNKEN)
            else:
                btn.config(bg="#eee", relief=tk.FLAT)

    def populate_tree(self, path, parent=''):
        """_summary_

        Args:
            path (_type_): _description_
            parent (str, optional): _description_. Defaults to ''.
        """
        try:
            items = os.listdir(path)
        except PermissionError:
            return

        for item in sorted(items):
            full_path = os.path.join(path, item)
            is_dir = os.path.isdir(full_path)
            icon = self.icons["folder"] if is_dir else self.icons["file"]

            node_id = self.tree.insert(parent, 'end', text=item, image=icon, open=False)
            self.tree.set(node_id, "fullpath", full_path)

            if is_dir and self.has_subitems(full_path):
                self.tree.insert(node_id, 'end', text='dummy', values=('dummy',))

    def has_subitems(self, path):
        """_summary_

        Args:
            path (_type_): _description_

        Returns:
            _type_: _description_
        """
        try:
            return any(os.listdir(path))
        except Exception:
            return False

    def on_treeview_open(self, event):
        """_summary_

        Args:
            event (_type_): _description_
        """
        node = self.tree.focus()
        path = self.tree.set(node, "fullpath")
        children = self.tree.get_children(node)

        dummy_found = any(self.tree.item(child, 'text') == 'dummy' for child in children)
        if dummy_found:
            self.tree.delete(*children)
            self.populate_tree(path, node)

    def on_treeview_double_click(self, event):
        """_summary_

        Args:
            event (_type_): _description_
        """
        item = self.tree.focus()
        path = self.get_full_path(item)
        if os.path.isfile(path):
            rel_path = os.path.relpath(path, self.base_data_path)
            print(f"[OPEN FILE] {rel_path}")

    def get_full_path(self, node):
        """_summary_

        Args:
            node (_type_): _description_

        Returns:
            _type_: _description_
        """
        return self.tree.set(node, "fullpath") if self.tree.set(node) else os.path.abspath('data')

    def toggle_debug(self, enable: bool):
        self.debug_state = enable
        new_level = logging.DEBUG if enable else logging.INFO
        self.logger.setLevel(new_level)
        self.logger.info(f"Debug mode set to {enable}")

if __name__ == "__main__":

    # Ensure logs folder exists
    LOG_FILE_PATH = os.path.join("data", "logs", "robot_ide.log")
    os.makedirs(os.path.dirname(LOG_FILE_PATH), exist_ok=True)

    # Load logging configuration
    CONFIG_PATH = os.path.join("data", "Configs", "logger_config.conf")
    logging.config.fileConfig(CONFIG_PATH)

    app = App()
    app.mainloop()


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
        key="M1",  # Use ID or name
        motor_params={
            "max_velocity": 800,
            "max_acceleration": 700
        }
    )
    arm_config.update_link(
        key="M2",
        motor_params={
            "max_velocity": 150,
            "max_acceleration": 650
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