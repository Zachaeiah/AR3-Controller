import tkinter as tk

class Tooltip:
    def __init__(self, widget: tk.Widget, text: str = "", delay=500):
        """
        Initialize a tooltip for a given widget.

        Args:
            widget (tk.Widget): The widget to attach the tooltip to.
            text (str): The tooltip text to display.
            delay (int, optional): Delay in milliseconds before showing the tooltip. Defaults to 500.
        """
        self.widget = widget
        self.text = text
        self.delay = delay  # Delay in milliseconds before showing tooltip
        self.tooltip_window = None
        self._after_id = None

        # Bind widget events to tooltip handlers
        self.widget.bind("<Enter>", self._schedule)
        self.widget.bind("<Leave>", self._hide)
        self.widget.bind("<Motion>", self._update_position)

    def _schedule(self, event=None):
        """
        Schedule the tooltip to appear after a delay when the mouse enters the widget.

        Args:
            event (tk.Event, optional): The event object (not used). Defaults to None.
        """
        self._after_id = self.widget.after(self.delay, self._show)

    def _show(self):
        """
        Show the tooltip near the widget after the delay.
        Prevents creating multiple tooltip windows.
        """
        if self.tooltip_window:
            return

        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 5

        # Create a new top-level window for the tooltip
        self.tooltip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)  # Remove window borders and title bar
        tw.wm_geometry(f"+{x}+{y}")

        # Create the label that displays the tooltip text
        label = tk.Label(
            tw,
            text=self.text,
            background="#ffffe0",  # Light yellow background
            relief="solid",
            borderwidth=1,
            font=("Segoe UI", 9),
            justify="left"
        )
        label.pack(ipadx=5, ipady=2)

    def _update_position(self, event):
        """
        Update the position of the tooltip window as the mouse moves.

        Args:
            event (tk.Event): The motion event object.
        """
        if self.tooltip_window:
            x = self.widget.winfo_rootx() + 20
            y = self.widget.winfo_rooty() + self.widget.winfo_height() + 5
            self.tooltip_window.wm_geometry(f"+{x}+{y}")

    def _hide(self, event=None):
        """
        Hide the tooltip when the mouse leaves the widget or tooltip area.

        Args:
            event (tk.Event, optional): The event object (not used). Defaults to None.
        """
        # Cancel scheduled tooltip display if it hasn't shown yet
        if self._after_id:
            self.widget.after_cancel(self._after_id)
            self._after_id = None
        # Destroy the tooltip window if it exists
        if self.tooltip_window:
            self.tooltip_window.destroy()
            self.tooltip_window = None

