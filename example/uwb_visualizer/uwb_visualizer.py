# -*- coding: utf-8 -*-
"""
UWB host visualizer.
- Reads aggregated serial lines from the master anchor (A0): T<id>,mask:..,seq:..,fail:..,range:(d0,..,d7),ancid:(..)
- Uses at least 3 enabled anchors with known (X, Y) coordinates in centimeters.
- Solves 2D multilateration with linear least squares and a short Gauss-Newton refinement.
"""

import os
import re
import sys
import time
import queue
import threading
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


MAX_ANCHORS = 8
TAG_TIMEOUT_S = 5.0
TAG_TRAIL_LEN = 30
RANGE_SMOOTH_ALPHA = 0.45
POSITION_SMOOTH_ALPHA = 0.45
RANGE_OUTLIER_JUMP_CM = 80.0
POSITION_OUTLIER_JUMP_CM = 80.0
POSITION_DEADBAND_CM = 5.0
LOGO_FILE_NAME = "logo.jpg"
PANEL_LOGO_FILE_NAME = "logo2.jpg"
SOURCE_LOGO_PATH = "D:/hhy/logo.jpg"
SOURCE_PANEL_LOGO_PATH = "D:/hhy/logo2.jpg"


def resource_path(file_name: str, fallback_path: str) -> str:
    """Returns a bundled resource path when packaged, otherwise the source file path."""
    bundled_dir = getattr(sys, "_MEIPASS", None)
    if bundled_dir:
        return os.path.join(bundled_dir, file_name)
    return fallback_path


LINE_REGEX = re.compile(
    r"T(?P<tid>\d+),mask:(?P<mask>[0-9A-Fa-f]+).*?range:\((?P<ranges>[^)]*)\),ancid:\((?P<ancids>[^)]*)\)"
)


@dataclass
class TagState:
    """Stores the latest position and track history for one tag."""
    tag_id: int
    position: Optional[np.ndarray] = None
    last_update: float = 0.0
    trail_x: list = field(default_factory=list)
    trail_y: list = field(default_factory=list)
    last_ranges: dict = field(default_factory=dict)
    smoothed_ranges: dict = field(default_factory=dict)


def parse_line(line: str):
    """Parses one serial line and returns (tag_id, {anchor_idx: distance_cm}) or None."""
    match = LINE_REGEX.search(line)
    if not match:
        return None

    try:
        tag_id = int(match.group("tid"))
        range_tokens = match.group("ranges").split(",")
        ancid_tokens = match.group("ancids").split(",")

        if len(range_tokens) != MAX_ANCHORS or len(ancid_tokens) != MAX_ANCHORS:
            return None

        ranges_by_anchor = {}

        for slot_idx in range(MAX_ANCHORS):
            anchor_idx = int(ancid_tokens[slot_idx].strip())

            if anchor_idx < 0:
                continue

            distance_cm = int(range_tokens[slot_idx].strip())

            if distance_cm <= 0:
                continue

            ranges_by_anchor[anchor_idx] = float(distance_cm)

        return tag_id, ranges_by_anchor
    except (ValueError, IndexError):
        return None


def solve_position_2d(anchor_positions: np.ndarray, distances: np.ndarray) -> Optional[np.ndarray]:
    """Solves the 2D tag position from anchor coordinates and measured distances."""
    num_anchors = anchor_positions.shape[0]

    if num_anchors < 3:
        return None

    reference_xy = anchor_positions[0]
    reference_d = distances[0]

    matrix_a = 2.0 * (anchor_positions[1:] - reference_xy)
    vector_b = (
        np.sum(anchor_positions[1:] ** 2, axis=1)
        - np.sum(reference_xy ** 2)
        + reference_d ** 2
        - distances[1:] ** 2
    )

    try:
        position_init, *_ = np.linalg.lstsq(matrix_a, vector_b, rcond=None)
    except np.linalg.LinAlgError:
        return None

    estimated_xy = position_init.astype(float)

    for _ in range(8):
        residual_vectors = estimated_xy - anchor_positions
        predicted_distances = np.linalg.norm(residual_vectors, axis=1)

        valid_mask = predicted_distances > 1e-6
        if not np.any(valid_mask):
            break

        residuals = predicted_distances[valid_mask] - distances[valid_mask]
        jacobian = residual_vectors[valid_mask] / predicted_distances[valid_mask, None]

        try:
            step, *_ = np.linalg.lstsq(jacobian, residuals, rcond=None)
        except np.linalg.LinAlgError:
            break

        estimated_xy = estimated_xy - step

        if np.linalg.norm(step) < 1e-3:
            break

    return estimated_xy


class SerialReader(threading.Thread):
    """Background serial reader that parses valid UWB lines into a queue."""

    def __init__(self, port_name: str, baudrate: int, output_queue: queue.Queue):
        super().__init__(daemon=True)
        self.port_name = port_name
        self.baudrate = baudrate
        self.output_queue = output_queue
        self.stop_event = threading.Event()
        self.serial_handle: Optional[serial.Serial] = None
        self.error_message: Optional[str] = None

    def run(self):
        try:
            self.serial_handle = serial.Serial(self.port_name, self.baudrate, timeout=0.2)
        except serial.SerialException as exc:
            self.error_message = str(exc)
            self.output_queue.put(("error", self.error_message))
            return

        buffer = b""

        while not self.stop_event.is_set():
            try:
                chunk = self.serial_handle.read(256)
            except serial.SerialException as exc:
                self.error_message = str(exc)
                self.output_queue.put(("error", self.error_message))
                break

            if chunk:
                buffer += chunk
                while b"\n" in buffer:
                    line_bytes, buffer = buffer.split(b"\n", 1)
                    line_text = line_bytes.decode("utf-8", errors="ignore").strip()

                    if not line_text:
                        continue

                    parsed = parse_line(line_text)
                    if parsed is not None:
                        self.output_queue.put(("data", parsed))

        if self.serial_handle is not None and self.serial_handle.is_open:
            self.serial_handle.close()

    def stop(self):
        """Requests the reader thread to stop."""
        self.stop_event.set()


class UwbVisualizerApp:
    """Main GUI with configuration controls on the left and a live map on the right."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("UWB")
        self.root.geometry("1100x680")

        self.anchor_entry_widgets = []
        self.anchor_enable_vars = []
        self.tag_states: dict[int, TagState] = {}
        self.serial_reader: Optional[SerialReader] = None
        self.data_queue: queue.Queue = queue.Queue()
        self.is_running = False
        self.smoothing_enabled_var = tk.BooleanVar(value=True)
        self.logo_photo = None
        self.window_icon_photo = None

        self._set_window_icon()
        self._build_layout()
        self._refresh_serial_ports()
        self.root.after(100, self._poll_serial_queue)

    def _set_window_icon(self):
        """Loads the company logo as the window title-bar icon."""
        icon_path = resource_path(LOGO_FILE_NAME, SOURCE_LOGO_PATH)
        if not os.path.exists(icon_path):
            return

        try:
            icon_image = Image.open(icon_path)
            icon_image.thumbnail((64, 64), Image.LANCZOS)
            self.window_icon_photo = ImageTk.PhotoImage(icon_image)
            self.root.iconphoto(True, self.window_icon_photo)
        except Exception:
            self.window_icon_photo = None

    def _build_layout(self):
        """Builds the main two-column layout."""
        main_pane = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True)

        config_frame = ttk.Frame(main_pane, padding=8)
        plot_frame = ttk.Frame(main_pane)
        main_pane.add(config_frame, weight=0)
        main_pane.add(plot_frame, weight=1)

        self._build_logo(config_frame)
        self._build_anchor_inputs(config_frame)
        self._build_serial_controls(config_frame)
        self._build_status_panel(config_frame)
        self._build_plot_area(plot_frame)

    def _build_logo(self, parent: ttk.Frame):
        """Loads and displays the company logo if the file exists."""
        logo_path = resource_path(PANEL_LOGO_FILE_NAME, SOURCE_PANEL_LOGO_PATH)
        if not os.path.exists(logo_path):
            return

        try:
            logo_image = Image.open(logo_path)
            logo_image.thumbnail((280, 120), Image.LANCZOS)
            self.logo_photo = ImageTk.PhotoImage(logo_image)
        except Exception:
            self.logo_photo = None
            return

        logo_label = ttk.Label(parent, image=self.logo_photo, anchor="center")
        logo_label.pack(fill=tk.X, pady=(0, 10))

    def _build_anchor_inputs(self, parent: ttk.Frame):
        """Creates the four anchor coordinate input rows."""
        group = ttk.LabelFrame(parent, text="Anchor coordinates (cm, at least 3 enabled)", padding=8)
        group.pack(fill=tk.X, pady=(0, 8))

        header_labels = ["Enabled", "Anchor", "X (cm)", "Y (cm)"]
        for col_idx, label_text in enumerate(header_labels):
            ttk.Label(group, text=label_text).grid(row=0, column=col_idx, padx=4, pady=2)

        default_coords = [(0, 0), (190, 0), (190, 120), (0, 120)]

        for anchor_idx in range(4):
            enable_var = tk.BooleanVar(value=True)
            self.anchor_enable_vars.append(enable_var)
            ttk.Checkbutton(group, variable=enable_var).grid(
                row=anchor_idx + 1, column=0, padx=4
            )
            ttk.Label(group, text=f"A{anchor_idx} (0x{0xA0 + anchor_idx:02X})").grid(
                row=anchor_idx + 1, column=1, sticky="w", padx=4
            )

            x_entry = ttk.Entry(group, width=10)
            y_entry = ttk.Entry(group, width=10)
            x_entry.insert(0, str(default_coords[anchor_idx][0]))
            y_entry.insert(0, str(default_coords[anchor_idx][1]))
            x_entry.grid(row=anchor_idx + 1, column=2, padx=4, pady=2)
            y_entry.grid(row=anchor_idx + 1, column=3, padx=4, pady=2)
            self.anchor_entry_widgets.append((x_entry, y_entry))

    def _build_serial_controls(self, parent: ttk.Frame):
        """Creates the serial port controls."""
        group = ttk.LabelFrame(parent, text="Serial settings", padding=8)
        group.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(group, text="COM port:").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        self.port_combobox = ttk.Combobox(group, state="readonly", width=22)
        self.port_combobox.grid(row=0, column=1, padx=4, pady=2)

        ttk.Button(group, text="Refresh", command=self._refresh_serial_ports).grid(
            row=0, column=2, padx=4, pady=2
        )

        ttk.Label(group, text="Baud rate:").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        self.baudrate_var = tk.StringVar(value="115200")
        baudrate_box = ttk.Combobox(
            group,
            textvariable=self.baudrate_var,
            values=["9600", "57600", "115200", "230400", "460800", "921600"],
            width=20,
        )
        baudrate_box.grid(row=1, column=1, padx=4, pady=2)

        ttk.Checkbutton(
            group,
            text="Smooth display",
            variable=self.smoothing_enabled_var,
        ).grid(row=2, column=0, columnspan=3, sticky="w", padx=4, pady=(6, 0))

        self.start_stop_button = ttk.Button(group, text="Start", command=self._toggle_serial)
        self.start_stop_button.grid(row=3, column=0, columnspan=3, sticky="ew", padx=4, pady=(6, 0))

    def _build_status_panel(self, parent: ttk.Frame):
        """Creates the status log panel."""
        group = ttk.LabelFrame(parent, text="Status", padding=8)
        group.pack(fill=tk.BOTH, expand=True)

        self.status_text = tk.Text(group, height=18, width=42, state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True)

    def _build_plot_area(self, parent: ttk.Frame):
        """Creates the live matplotlib map."""
        self.figure = Figure(figsize=(6, 6), dpi=100)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_title("UWB Live Position (cm)")
        self.axes.set_xlabel("X (cm)")
        self.axes.set_ylabel("Y (cm)")
        self.axes.set_aspect("equal", adjustable="datalim")
        self.axes.grid(True, linestyle="--", alpha=0.5)

        self.canvas = FigureCanvasTkAgg(self.figure, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _refresh_serial_ports(self):
        """Refreshes the available serial port list."""
        available_ports = [port_info.device for port_info in serial.tools.list_ports.comports()]
        self.port_combobox["values"] = available_ports
        if available_ports:
            self.port_combobox.current(0)

    def _toggle_serial(self):
        """Starts or stops serial collection."""
        if self.is_running:
            self._stop_serial()
        else:
            self._start_serial()

    def _collect_anchor_positions(self):
        """Reads enabled anchor coordinates from the UI."""
        positions = {}

        for anchor_idx, (enable_var, (x_entry, y_entry)) in enumerate(
            zip(self.anchor_enable_vars, self.anchor_entry_widgets)
        ):
            if not enable_var.get():
                continue

            try:
                x_value = float(x_entry.get())
                y_value = float(y_entry.get())
            except ValueError:
                raise ValueError(f"A{anchor_idx} coordinates must be valid numbers")

            positions[anchor_idx] = np.array([x_value, y_value])

        return positions

    def _start_serial(self):
        """Starts the serial reader."""
        try:
            self.anchor_positions = self._collect_anchor_positions()
        except ValueError as exc:
            messagebox.showerror("Coordinate error", str(exc))
            return

        if len(self.anchor_positions) < 3:
            messagebox.showerror("Not enough anchors", "Enable at least 3 anchors and enter their coordinates")
            return

        selected_port = self.port_combobox.get()
        if not selected_port:
            messagebox.showerror("No serial port", "Please select a COM port")
            return

        try:
            baudrate_value = int(self.baudrate_var.get())
        except ValueError:
            messagebox.showerror("Baud rate error", "Baud rate must be a number")
            return

        self.tag_states.clear()
        self.data_queue = queue.Queue()
        self.serial_reader = SerialReader(selected_port, baudrate_value, self.data_queue)
        self.serial_reader.start()

        self.is_running = True
        self.start_stop_button.config(text="Stop")
        self._append_status(f"Opened {selected_port} @ {baudrate_value}")

    def _stop_serial(self):
        """Stops the serial reader."""
        if self.serial_reader is not None:
            self.serial_reader.stop()
            self.serial_reader.join(timeout=1.0)
            self.serial_reader = None

        self.is_running = False
        self.start_stop_button.config(text="Start")
        self._append_status("Collection stopped")

    def _poll_serial_queue(self):
        """Polls queued serial data from the Tk main thread."""
        while True:
            try:
                message_kind, payload = self.data_queue.get_nowait()
            except queue.Empty:
                break

            if message_kind == "error":
                self._append_status(f"Serial error: {payload}")
                self._stop_serial()
                continue

            if message_kind == "data":
                tag_id, ranges_by_anchor = payload
                self._update_tag(tag_id, ranges_by_anchor)

        self._redraw_plot()
        self.root.after(100, self._poll_serial_queue)

    def _update_tag(self, tag_id: int, ranges_by_anchor: dict):
        """Updates one tag position from one parsed serial frame."""
        tag_state = self.tag_states.setdefault(tag_id, TagState(tag_id=tag_id))
        tag_state.last_ranges = dict(ranges_by_anchor)

        if self.smoothing_enabled_var.get():
            for anchor_idx, raw_distance in ranges_by_anchor.items():
                if anchor_idx not in self.anchor_positions:
                    continue

                previous_distance = tag_state.smoothed_ranges.get(anchor_idx)
                if previous_distance is None:
                    tag_state.smoothed_ranges[anchor_idx] = raw_distance
                    continue

                distance_jump = abs(raw_distance - previous_distance)
                if distance_jump > RANGE_OUTLIER_JUMP_CM:
                    continue

                tag_state.smoothed_ranges[anchor_idx] = (
                    previous_distance * (1.0 - RANGE_SMOOTH_ALPHA)
                    + raw_distance * RANGE_SMOOTH_ALPHA
                )

            active_ranges = tag_state.smoothed_ranges
        else:
            active_ranges = ranges_by_anchor

        usable_anchor_indices = [
            idx for idx in sorted(active_ranges.keys()) if idx in self.anchor_positions
        ]

        if len(usable_anchor_indices) < 3:
            self._append_status(
                f"T{tag_id} has only {len(usable_anchor_indices)} valid anchors, skipped"
            )
            return

        anchor_xy_matrix = np.array(
            [self.anchor_positions[idx] for idx in usable_anchor_indices]
        )
        distance_vector = np.array(
            [active_ranges[idx] for idx in usable_anchor_indices]
        )

        estimated_position = solve_position_2d(anchor_xy_matrix, distance_vector)
        if estimated_position is None:
            self._append_status(f"T{tag_id} position solve failed")
            return

        if tag_state.position is None:
            display_position = estimated_position
        else:
            position_jump = np.linalg.norm(estimated_position - tag_state.position)
            if position_jump > POSITION_OUTLIER_JUMP_CM:
                return

            if not self.smoothing_enabled_var.get():
                display_position = estimated_position
            elif position_jump < POSITION_DEADBAND_CM:
                display_position = tag_state.position
            else:
                display_position = (
                    tag_state.position * (1.0 - POSITION_SMOOTH_ALPHA)
                    + estimated_position * POSITION_SMOOTH_ALPHA
                )

        tag_state.position = display_position
        tag_state.last_update = time.time()

        tag_state.trail_x.append(float(display_position[0]))
        tag_state.trail_y.append(float(display_position[1]))
        if len(tag_state.trail_x) > TAG_TRAIL_LEN:
            tag_state.trail_x = tag_state.trail_x[-TAG_TRAIL_LEN:]
            tag_state.trail_y = tag_state.trail_y[-TAG_TRAIL_LEN:]

    def _redraw_plot(self):
        """Redraws anchors, tags, and tag tracks."""
        self.axes.clear()
        self.axes.set_title("UWB Live Position (cm)")
        self.axes.set_xlabel("X (cm)")
        self.axes.set_ylabel("Y (cm)")
        self.axes.set_aspect("equal", adjustable="datalim")
        self.axes.grid(True, linestyle="--", alpha=0.5)

        if not hasattr(self, "anchor_positions"):
            self.canvas.draw_idle()
            return

        for anchor_idx, anchor_xy in self.anchor_positions.items():
            self.axes.scatter(
                anchor_xy[0], anchor_xy[1], marker="^", s=120, c="tab:blue", zorder=3
            )
            self.axes.annotate(
                f"A{anchor_idx}",
                xy=(anchor_xy[0], anchor_xy[1]),
                xytext=(6, 6),
                textcoords="offset points",
                color="tab:blue",
            )

        current_time = time.time()
        active_tag_ids = []
        for tag_id, tag_state in self.tag_states.items():
            if tag_state.position is None:
                continue
            if current_time - tag_state.last_update > TAG_TIMEOUT_S:
                continue

            active_tag_ids.append(tag_id)

            self.axes.plot(
                tag_state.trail_x, tag_state.trail_y, "-", alpha=0.4, linewidth=1
            )
            self.axes.scatter(
                tag_state.position[0],
                tag_state.position[1],
                marker="o",
                s=90,
                c="tab:red",
                zorder=4,
            )
            self.axes.annotate(
                f"T{tag_id}",
                xy=(tag_state.position[0], tag_state.position[1]),
                xytext=(6, -10),
                textcoords="offset points",
                color="tab:red",
            )

        all_x = [xy[0] for xy in self.anchor_positions.values()]
        all_y = [xy[1] for xy in self.anchor_positions.values()]
        for tag_id in active_tag_ids:
            all_x.append(self.tag_states[tag_id].position[0])
            all_y.append(self.tag_states[tag_id].position[1])

        if all_x and all_y:
            margin = max(50.0, 0.1 * max(max(all_x) - min(all_x), max(all_y) - min(all_y)))
            self.axes.set_xlim(min(all_x) - margin, max(all_x) + margin)
            self.axes.set_ylim(min(all_y) - margin, max(all_y) + margin)

        self.canvas.draw_idle()

    def _append_status(self, text: str):
        """Appends one line to the status log."""
        self.status_text.config(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.status_text.insert(tk.END, f"[{timestamp}] {text}\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)

    def on_close(self):
        """Cleans up when the main window closes."""
        if self.is_running:
            self._stop_serial()
        self.root.destroy()


def main():
    """Application entry point."""
    root = tk.Tk()
    app = UwbVisualizerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
