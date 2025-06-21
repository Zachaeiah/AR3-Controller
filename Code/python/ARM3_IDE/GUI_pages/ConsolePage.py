from .GUI_template import PageBase
import tkinter as tk
from tkinter import ttk
from typing import Optional
import logging
import os

from datetime import datetime

class ConsolePage(PageBase):
    """_summary_

    Args:
        PageBase (_type_): _description_
    """
    PAGE_NAME = "Console"
    SEL_BTN_TT_MSGS =   """CLI controll for the robot"""

    def __init__(self, parent: tk.Widget, controller, logger: Optional[logging.Logger] = None):
        """
        _summary_

        Args:
            parent (Widget): The parent container, typically a frame from the main app.
            controller (App): The main application controller used for accessing shared resources.
            logger (Optional[logging.Logger]): Logger instance for logging debug/info/errors.
        """
        super().__init__(parent, controller, title=self.PAGE_NAME)

        self.controller = controller
        self.logger: logging.Logger = logger

        self.command_history: list = []
        self.history_index: int = -1
        self.timestamp_enabled: tk.BooleanVar = tk.BooleanVar(value=True)

        # Console Output Text Box with Scrollbar
        self.output_text: tk.Text = tk.Text(self, wrap=tk.WORD, height=20, state=tk.DISABLED)
        scrollbar: ttk.Scrollbar = ttk.Scrollbar(self, command=self.output_text.yview)
        self.output_text.configure(yscrollcommand=scrollbar.set)

        self.output_text.grid(row=0, column=0, columnspan=3, sticky="nsew", padx=10, pady=10)
        scrollbar.grid(row=0, column=3, sticky="ns", pady=10)

        # Command Entry and Send Button
        self.command_entry: tk.Entry = tk.Entry(self)
        self.command_entry.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 10))
        self.command_entry.bind("<Return>", self.send_command)
        self.command_entry.bind("<Up>", self.previous_command)
        self.command_entry.bind("<Down>", self.next_command)
        self.TOOLTIPS[self.command_entry] = "Entry for Commands"

        self.send_button: tk.Button = tk.Button(self, text="Send", command=self.send_command)
        self.send_button.grid(row=1, column=1, sticky="ew", padx=(0, 10), pady=(0, 10))
        self.TOOLTIPS[self.send_button] = "Send Command in entry to robot"

        self.clear_button: tk.Button = tk.Button(self, text="Clear", command=self.clear_console)
        self.clear_button.grid(row=1, column=2, sticky="ew", padx=(0, 10), pady=(0, 10))
        self.TOOLTIPS[self.clear_button] = "Will clean CLI textt"

        self.save_button: tk.Button = tk.Button(self, text="Save Log", command=self.save_log)
        self.save_button.grid(row=2, column=2, sticky="ew", padx=(0, 10), pady=(0, 10))
        self.TOOLTIPS[self.save_button] = "Save CLI text"

        self.timestamp_check: tk.Checkbutton = tk.Checkbutton(self, text="Timestamps", variable=self.timestamp_enabled)
        self.timestamp_check.grid(row=2, column=0, sticky="w", padx=10)
        self.TOOLTIPS[self.timestamp_check] = "add timestamp to cli ms"

        # Connection Status Label (Status Bar)
        self.status_label: tk.Label = tk.Label(self, text="Status: disonnected", anchor="w", fg="red")
        self.status_label.grid(row=3, column=0, columnspan=3, sticky="ew", padx=10, pady=(5, 10))
        self.TOOLTIPS[self.status_label] = "the Connection status of the robot to the CL"

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.bind_all("<Control-s>", lambda e: self.save_log())
        self.bind_all("<Control-l>", lambda e: self.clear_console())

    def log_to_console(self, message: str)->None:
        self.logger.info(f"message to log: {message}")

        self.output_text.configure(state=tk.NORMAL)
        self.output_text.insert(tk.END, message + "\n")
        self.output_text.see(tk.END)
        self.output_text.configure(state=tk.DISABLED)

    def send_command(self, event=None) ->None:
        command: str = self.command_entry.get().strip()
        if not command:
            return

        self.command_history.append(command)
        self.history_index: int = len(self.command_history)

        self.log_to_console(f">>> {command}")
        self.command_entry.delete(0, tk.END)

        try:
            response:str  = f"[Simulated Robot Response] {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
            if self.timestamp_enabled.get():
                response: str = f"[{datetime.now().strftime('%H:%M:%S')}] {response}"

            self.logger.info(f"response: {response}")
            self.log_to_console(response)
        except Exception as e:
            error_msg:str = f"[Error] {e}"
            self.logger.warning(error_msg)
            self.log_to_console(error_msg)

    def previous_command(self, event)->None:
        if self.history_index > 0:
            self.history_index -= 1
            self.command_entry.delete(0, tk.END)
            self.command_entry.insert(0, self.command_history[self.history_index])

    def next_command(self, event)->None:
        if self.history_index < len(self.command_history) - 1:
            self.history_index += 1
            self.command_entry.delete(0, tk.END)
            self.command_entry.insert(0, self.command_history[self.history_index])
        else:
            self.command_entry.delete(0, tk.END)
            self.history_index: int = len(self.command_history)

    def clear_console(self) ->None:
        self.output_text.configure(state=tk.NORMAL)
        self.output_text.delete("1.0", tk.END)
        self.output_text.configure(state=tk.DISABLED)

    def save_log(self) ->None :
        log_text:str = self.output_text.get("1.0", tk.END).strip()
        log_dir:str = os.path.join("data", "logs")
        os.makedirs(log_dir, exist_ok=True)

        base_filename:str = datetime.now().strftime("console_%Y%m%d")
        filename:str = os.path.join(log_dir, f"{base_filename}.txt")
        counter:int = 1

        while os.path.exists(filename):
            filename: str = os.path.join(log_dir, f"{base_filename}_{counter}.txt")
            counter += 1

        with open(filename, "w") as f:
            f.write(log_text)

        self.logger.info(f"Console log saved to {filename}")
        

    def on_show(self) -> None:
        self.logger.debug(f"Switching to {self.PAGE_NAME}")

        #update connectshion status UI
        if self.controller.Robot_connect_state:
            self.status_label.config(text="Status: Connected", fg="green")
        else:
            self.status_label.config(text="Status: Disconnected", fg="red")

        self.command_entry.focus_set()