"""Interactive desktop control panel for the guarded RAICOM teleop node.

The panel publishes key events only.  ``keyboard_input`` remains the sole
publisher of ``/raicom/teleop_cmd`` and the watchdog remains the sole output
to ``/controller/cmd_vel``.
"""

from __future__ import annotations

import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


KEYS = {"w", "s", "a", "d", "q", "e"}
CLICK_PULSE_SEC = 0.45
MIN_HOLD_SEC = 0.15
BUTTONS = (
    ("W\n前进", "w", 0, 1),
    ("A\n左移", "a", 1, 0),
    ("S\n后退", "s", 1, 1),
    ("D\n右移", "d", 1, 2),
    ("Q\n左转", "q", 2, 0),
    ("E\n右转", "e", 2, 2),
)


class KeyEventPublisher(Node):
    def __init__(self) -> None:
        super().__init__("manual_control_panel")
        self.navigation_active = False
        self.publisher = self.create_publisher(String, "/raicom/teleop_key", 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.navigation_subscription = self.create_subscription(
            Bool, "/raicom/navigation_active", self._on_navigation_active, qos)

    def _on_navigation_active(self, message: Bool) -> None:
        self.navigation_active = bool(message.data)

    def send(self, key: str) -> None:
        message = String()
        message.data = key
        self.publisher.publish(message)


class ControlPanel:
    def __init__(self, node: KeyEventPublisher) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("RAICOM-Controls")
        self.root.geometry("430x330+1460+660")
        self.root.resizable(False, False)
        self.root.attributes("-topmost", True)
        self.root.configure(bg="#202124")
        self.held_key: str | None = None
        self.repeat_job: str | None = None
        self.click_stop_job: str | None = None
        self.press_started_at: float | None = None
        self.status_var = tk.StringVar(value="状态：已停车")

        tk.Label(
            self.root,
            text="RAICOM 手动建图控制",
            font=("Sans", 16, "bold"),
            fg="#7dd3fc",
            bg="#202124",
        ).pack(pady=(12, 4))
        tk.Label(
            self.root,
            text="短按方向按钮微动 0.45 秒；按住持续移动，松开即停车。",
            font=("Sans", 10),
            fg="#cbd5e1",
            bg="#202124",
        ).pack(pady=(0, 8))
        tk.Label(
            self.root,
            textvariable=self.status_var,
            font=("Sans", 10, "bold"),
            fg="#86efac",
            bg="#202124",
        ).pack(pady=(0, 4))

        grid = tk.Frame(self.root, bg="#202124")
        grid.pack()
        for label, key, row, column in BUTTONS:
            button = tk.Button(
                grid,
                text=label,
                width=8,
                height=2,
                font=("Sans", 12, "bold"),
                fg="white",
                bg="#334155",
                activebackground="#0ea5e9",
                relief="raised",
                bd=2,
            )
            button.grid(row=row, column=column, padx=5, pady=4)
            button.bind("<ButtonPress-1>", lambda _event, k=key: self.start_hold(k))
            button.bind("<ButtonRelease-1>", lambda _event: self.stop_hold())

        stop = tk.Button(
            self.root,
            text="SPACE / 立即停车",
            width=24,
            height=1,
            font=("Sans", 12, "bold"),
            fg="white",
            bg="#b91c1c",
            activebackground="#ef4444",
        )
        stop.pack(pady=(7, 5))
        stop.configure(command=lambda: self.stop_hold(immediate=True))

        tk.Label(
            self.root,
            text="本窗已是可操作面板；不要按 M。RViz 中灰白小车就是当前位置。",
            font=("Sans", 10),
            fg="#fbbf24",
            bg="#202124",
        ).pack(pady=(0, 5))

        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.bind("<space>", lambda _event: self.stop_hold(immediate=True))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        # The launch terminal is often raised after this window starts.  Keep
        # the control panel visible and focused so a click/keyboard event is
        # not accidentally sent to RViz or an unrelated terminal.
        self.root.after(400, self.bring_to_front)
        self.root.after(30, self.spin_ros)

    def bring_to_front(self) -> None:
        if self.root.winfo_exists():
            self.root.deiconify()
            self.root.lift()
            self.root.attributes("-topmost", True)
            self.root.focus_force()

    def on_key_press(self, event: tk.Event) -> None:
        key = str(event.keysym).lower()
        if key in KEYS and self.held_key != key:
            self.start_hold(key)

    def on_key_release(self, event: tk.Event) -> None:
        key = str(event.keysym).lower()
        if key in KEYS:
            self.stop_hold()

    def start_hold(self, key: str) -> None:
        if key not in KEYS:
            return
        if self.node.navigation_active:
            self.status_var.set("状态：导航中，仅 SPACE 停车有效")
            return
        if self.click_stop_job is not None:
            self.root.after_cancel(self.click_stop_job)
            self.click_stop_job = None
        self.held_key = key
        self.press_started_at = time.monotonic()
        self.status_var.set(f"状态：正在发送 {key.upper()}（松开停车）")
        self.node.send(key)
        if self.repeat_job is None:
            self.repeat_job = self.root.after(80, self.repeat_key)

    def repeat_key(self) -> None:
        self.repeat_job = None
        if self.held_key is not None:
            self.node.send(self.held_key)
            self.repeat_job = self.root.after(80, self.repeat_key)

    def stop_hold(self, *, immediate: bool = False) -> None:
        if self.click_stop_job is not None:
            self.root.after_cancel(self.click_stop_job)
            self.click_stop_job = None
        started_at = self.press_started_at
        was_held = self.held_key is not None
        self.held_key = None
        self.press_started_at = None
        if self.repeat_job is not None:
            self.root.after_cancel(self.repeat_job)
            self.repeat_job = None
        if (
            not immediate
            and was_held
            and started_at is not None
            and time.monotonic() - started_at < MIN_HOLD_SEC
        ):
            remaining = max(0.0, CLICK_PULSE_SEC - (time.monotonic() - started_at))
            self.click_stop_job = self.root.after(
                max(1, int(remaining * 1000)), self.finish_click_pulse
            )
            return
        self.node.send(" ")
        self.status_var.set("状态：已停车")

    def finish_click_pulse(self) -> None:
        self.click_stop_job = None
        self.node.send(" ")
        self.status_var.set("状态：已停车（短按微动完成）")

    def spin_ros(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if self.node.navigation_active and self.held_key is None:
                self.status_var.set("状态：导航中，仅 SPACE 停车有效")
            self.root.after(30, self.spin_ros)

    def close(self) -> None:
        self.stop_hold(immediate=True)
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = KeyEventPublisher()
    panel = ControlPanel(node)
    panel.run()


if __name__ == "__main__":
    main()
