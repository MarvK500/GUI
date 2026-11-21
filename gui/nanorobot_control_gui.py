#!/usr/bin/env python3
"""NanoRobot Control Center — modern Flet desktop UI for the nanorobot UART controller."""

from __future__ import annotations

import argparse
import asyncio
import csv
import inspect
import math
import queue
import random
import struct
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable

try:
    import flet as ft
except ImportError:  # Friendly error is shown from main().
    ft = None  # type: ignore[assignment]

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:
    serial = None
    list_ports = None

import nanorobot_control_tk_legacy as proto


# Flet 0.28 and the reorganized 0.80+ API expose these helpers in different
# namespaces. Keep one source compatible with both layouts so an existing
# installation does not fail before the dependency installer can be rerun.
if ft is not None:
    PADDING_SYMMETRIC = (
        ft.Padding.symmetric if hasattr(ft.Padding, "symmetric") else ft.padding.symmetric
    )
    BORDER_ALL = ft.Border.all if hasattr(ft.Border, "all") else ft.border.all
    ALIGN_CENTER = (
        ft.Alignment.CENTER if hasattr(ft.Alignment, "CENTER") else ft.alignment.center
    )
    ALIGN_TOP_LEFT = (
        ft.Alignment.TOP_LEFT if hasattr(ft.Alignment, "TOP_LEFT") else ft.alignment.top_left
    )
    ALIGN_BOTTOM_RIGHT = (
        ft.Alignment.BOTTOM_RIGHT
        if hasattr(ft.Alignment, "BOTTOM_RIGHT")
        else ft.alignment.bottom_right
    )


def dropdown_event(handler: Callable[..., Any]) -> dict[str, Any]:
    """Return the Dropdown event keyword used by the installed Flet API."""
    event_name = "on_select" if "on_select" in inspect.signature(ft.Dropdown).parameters else "on_change"
    return {event_name: handler}


APP_VERSION = "4.9 Flet"
REQUIREMENTS_FILE = Path(__file__).resolve().with_name("requirements.txt")
INSTALL_COMMAND = f'python -m pip install -r "{REQUIREMENTS_FILE}"'
PRIMARY = "#7C3AED"
PRIMARY_DARK = "#5B21B6"
PRIMARY_SOFT = "#F3E8FF"
BACKGROUND = "#F7F5FC"
SURFACE = "#FFFFFF"
BORDER = "#E7E0F4"
TEXT = "#2B2340"
MUTED = "#756B89"
GREEN = "#059669"
GREEN_SOFT = "#D1FAE5"
AMBER = "#D97706"
AMBER_SOFT = "#FEF3C7"
RED = "#DC2626"
RED_SOFT = "#FEE2E2"
BLUE = "#2563EB"
MONO = "Consolas"


# The UI intentionally exposes only the analyzed current-firmware profile.
ACTIVE_PROFILE = proto.PROFILE_CURRENT
BLOCKED_CURRENT_FIRMWARE: dict[int, str] = {}

MOTOR_COMMAND_IDS = frozenset((0x0E, 0x0F, 0x10))
COIL_COMMAND_IDS = frozenset(range(0x00, 0x0D))
COIL_STOP_STATUSES = frozenset((0x0E, 0x13, 0x30, 0x31, 0x32, 0x33, 0x35, 0x36, 0x37))
STARTUP_STATUSES = range(0x1E, 0x2E)
STARTUP_SENSOR_COUNT = 8
EXPECTED_RESPONSE_IDS: dict[int, int] = {
    command_id: 0x80 + command_id for command_id in range(0x19)
}
AMBIENT_MAGNETOMETER_MAX_PERIOD_S = 60
COIL_MAGNETOMETER_MAX_FREQUENCY_HZ = 250
SUCCESS_STATUS = 0x77
NOTIFICATION_IDS = frozenset((0x9C, 0x9D, 0xFF))
UNIT_LIMITS: dict[int, tuple[str, int]] = {0: ("mV", 12000), 1: ("mA", 1000), 2: ("µT", 6000)}
FIRMWARE_DEFAULT_UNIT = 0
UNIT_LIMITED_COMMANDS = frozenset((0x01, 0x02, 0x03, 0x04, 0x06))
STANDARD_RESPONSE_TIMEOUT = 25
MOTOR_STEP_TIME_S = 0.012
MOTOR_TIMEOUT_FACTOR = 1.25
MOTOR_TIMEOUT_EXTRA_S = 5.0
EXCHANGE_MOTOR_TRAVEL_STEPS = 6000
EXCHANGE_MOTOR_RESET_STEPS = 6000 + 580
FOCUS_MOTOR_RESET_STEPS = 400 + 200
MAX_COMMAND_QUEUE = 50
MIN_FRAME_GAP_S = 0.005
CHECK_RX_CRC = False


def motor_steps(command_id: int, payload: bytes) -> int:
    if command_id == 0x0F and len(payload) >= 4:
        return int.from_bytes(payload[2:4], "big")
    if command_id == 0x0E and payload:
        return EXCHANGE_MOTOR_RESET_STEPS if payload[0] == 0 else FOCUS_MOTOR_RESET_STEPS
    return EXCHANGE_MOTOR_TRAVEL_STEPS


def startup_sensor(status: int) -> tuple[str, bool]:
    if status <= 0x25:
        return f"Magnetometer {(status - 0x1E) % 4}", status >= 0x22
    return f"Thermometer {(status - 0x26) % 4}", status >= 0x2A


@dataclass
class QueuedCommand:
    command_id: int
    sequence: int
    payload: bytes
    wire: bytes
    description: str
    notify: bool = True
    enforce_delay: bool = True
    source: str = "manual"
    bypass_gate: bool = False

    @property
    def expected_response_id(self) -> int:
        return EXPECTED_RESPONSE_IDS[self.command_id]

    @property
    def response_timeout(self) -> float:
        if self.command_id not in MOTOR_COMMAND_IDS:
            return STANDARD_RESPONSE_TIMEOUT
        motor_time = motor_steps(self.command_id, self.payload) * MOTOR_STEP_TIME_S
        return max(STANDARD_RESPONSE_TIMEOUT, motor_time * MOTOR_TIMEOUT_FACTOR + MOTOR_TIMEOUT_EXTRA_S)


COMMAND_META: dict[int, tuple[str, str, str]] = {
    0x00: ("Input units", "Coils", "Select millivolts, milliamps or magnetic-field units."),
    0x01: ("Async single channel", "Coils", "Configure one independent X, Y or Z waveform."),
    0x02: ("Async all channels", "Coils", "Configure independent X, Y and Z waveforms."),
    0x03: ("Sync single channel", "Coils", "Configure one channel using the common synchronous frequency."),
    0x04: ("Sync all channels", "Coils", "Configure all channels with a shared frequency."),
    0x05: ("Synchronous frequency", "Coils", "Change the common synchronous frequency."),
    0x06: ("Rotating magnetic field", "Coils", "Set azimuth, elevation, amplitude and rotation frequency."),
    0x07: ("Coil outputs", "Coils", "Enable or disable selected coil outputs."),
    0x08: ("Channel synchronization", "Coils", "Restart the asynchronous channel time base."),
    0x09: ("Field calibration matrix", "Calibration", "Set the 3 × 3 field-to-current conversion matrix."),
    0x0A: ("Current-sensor zero", "Calibration", "Set the sensor voltage corresponding to zero current."),
    0x0B: ("Current-sensor filter", "Calibration", "Set the current-sensor IIR alpha value."),
    0x0C: ("PI controller", "Calibration", "Configure the PI controller and output limits."),
    0x0D: ("Connection check", "Communication", "Request the firmware Hello response."),
    0x0E: ("Reference motor", "Motors", "Move a stepper motor toward its mechanical reference."),
    0x0F: ("Move motor", "Motors", "Perform a relative stepper-motor movement."),
    0x10: ("Move to sample", "Motors", "Move the exchange motor to a stored sample position."),
    0x11: ("Save sample position", "Motors", "Store the current exchange-motor position in RAM."),
    0x12: ("Fan", "Peripherals", "Turn the fan on or off."),
    0x13: ("Lighting", "Peripherals", "Control backlights or the UV bio-killer output."),
    0x14: ("Start thermometer", "Sensors", "Start one-shot or periodic temperature measurement."),
    0x15: ("Stop thermometer", "Sensors", "Stop continuous measurement on one thermometer."),
    0x16: ("Magnetometer temperature", "Sensors", "Read the selected magnetometer temperature."),
    0x17: ("Start magnetometer", "Sensors", "Measure the magnetic field once (rate 0) or start periodic notifications."),
    0x18: ("Stop magnetometer", "Sensors", "Stop periodic magnetic field notifications."),
}


FIELD_LABELS = {
    "input_type": "Input unit",
    "axis": "Axis",
    "amplitude": "Amplitude",
    "amplitude_x": "Amplitude X",
    "amplitude_y": "Amplitude Y",
    "amplitude_z": "Amplitude Z",
    "frequency": "Frequency",
    "frequency_x": "Frequency X",
    "frequency_y": "Frequency Y",
    "frequency_z": "Frequency Z",
    "offset": "Offset",
    "offset_x": "Offset X",
    "offset_y": "Offset Y",
    "offset_z": "Offset Z",
    "phase": "Phase",
    "phase_x": "Phase X",
    "phase_y": "Phase Y",
    "phase_z": "Phase Z",
    "ramp": "Ramp",
    "ramp_x": "Ramp X",
    "ramp_y": "Ramp Y",
    "ramp_z": "Ramp Z",
    "azimuth": "Azimuth",
    "elevation": "Elevation",
    "output": "Output",
    "state": "State",
    "sync": "Synchronize",
    "zero": "Zero voltage",
    "iir": "IIR alpha",
    "kp": "Kp",
    "ki": "Ki",
    "low": "Lower limit",
    "high": "Upper limit",
    "motor": "Motor",
    "direction": "Direction",
    "steps": "Steps",
    "sample": "Sample",
    "action": "State",
    "light": "Light",
    "sensor": "Sensor",
    "period": "Period",
    "magnetometer": "Magnetometer",
    "rate": "Rate (0 = single measurement)",
    "xx": "XX", "xy": "XY", "xz": "XZ",
    "yx": "YX", "yy": "YY", "yz": "YZ",
    "zx": "ZX", "zy": "ZY", "zz": "ZZ",
}


CHOICE_TRANSLATIONS = {
    "Všechny": "All",
    "STOP / vypnout": "STOP / disabled",
    "RUN / zapnout": "RUN / enabled",
    "Výměnný motor": "Exchange motor",
    "Focus motor": "Focus motor",
    "Nahoru / vpřed": "Forward / up",
    "Dolů / vzad": "Reverse / down",
    "Vypnout": "Off",
    "Zapnout": "On",
    "Podsvit 0": "Backlight 0",
    "Podsvit 1": "Backlight 1",
    "UV Bio-killer": "UV bio-killer",
    "Ano": "Yes",
}


STATUS_EN = {
    0x01: "Input queue full", 0x02: "Input queue empty", 0x03: "UART failure",
    0x04: "UART TX error", 0x05: "UART TX busy", 0x06: "UART timeout",
    0x07: "Frame start not found", 0x08: "Frame end not found", 0x09: "Unknown command",
    0x0A: "Invalid payload length", 0x0B: "Invalid CRC", 0x0C: "Invalid frame length",
    0x0D: "Response too long", 0x0E: "Null pointer", 0x0F: "Output queue full",
    0x10: "Output queue empty", 0x11: "L99SM81V SPI error",
    0x12: "STM32 SPI busy", 0x13: "Not initialized", 0x14: "Invalid parameter",
    0x15: "Stepper motor disabled", 0x16: "L99SM81V functional error",
    0x17: "L99SM81V device error", 0x18: "L99SM81V global warning", 0x19: "L99SM81V reset",
    0x1A: "STM32 SPI error", 0x1B: "L99SM81V probably disabled",
    0x1C: "STM32 I²C error", 0x1D: "STM32 I²C busy",
    0x1E: "Magnetometer 0: FAILED", 0x1F: "Magnetometer 1: FAILED",
    0x20: "Magnetometer 2: FAILED", 0x21: "Magnetometer 3: FAILED",
    0x22: "Magnetometer 0: OK", 0x23: "Magnetometer 1: OK",
    0x24: "Magnetometer 2: OK", 0x25: "Magnetometer 3: OK",
    0x26: "Thermometer 0: FAILED", 0x27: "Thermometer 1: FAILED",
    0x28: "Thermometer 2: FAILED", 0x29: "Thermometer 3: FAILED",
    0x2A: "Thermometer 0: OK", 0x2B: "Thermometer 1: OK",
    0x2C: "Thermometer 2: OK", 0x2D: "Thermometer 3: OK",
    0x2F: "Coils not initialized", 0x30: "Invalid input type", 0x31: "Invalid channel",
    0x32: "Invalid coil output", 0x33: "Invalid output state", 0x35: "Invalid PI gain",
    0x36: "Invalid PI limits", 0x37: "Invalid coil state", 0x38: "General error",
    0x39: "Magnetometer not initialized", 0x77: "SUCCESS",
}


def command_title(command_id: int) -> str:
    return COMMAND_META.get(command_id, (f"Command 0x{command_id:02X}", "Other", ""))[0]


def response_name(response_id: int, command_id: int | None = None) -> str:
    if command_id in COMMAND_META:
        return command_title(command_id)
    if response_id == 0xFF:
        return "System notification"
    if response_id == 0x9C:
        return "Thermometer notification"
    if response_id == 0x9D:
        return "Magnetometer notification"
    matching_commands = [cmd for cmd, rsp in EXPECTED_RESPONSE_IDS.items() if rsp == response_id]
    if len(matching_commands) == 1:
        return command_title(matching_commands[0])
    return f"Response 0x{response_id:02X}"


def normal_response_fields(body: bytes) -> tuple[int, int, int, bytes] | None:
    """Return ID, SEQ, status and payload for one complete command response."""
    # Do not whitelist a particular response ID and do not compare SEQ here.
    # Only the normal-response structure and its exact length matter.
    if len(body) < 9 or not 0x80 <= body[0] < 0xFF or body[0] in NOTIFICATION_IDS:
        return None
    payload_length = int.from_bytes(body[3:5], "big")
    expected_length = 5 + payload_length + 4
    if expected_length != len(body):
        return None
    return body[0], body[1], body[2], body[5:5 + payload_length]


def notification_fields(body: bytes) -> tuple[int, int, bytes] | None:
    """Return ID, status and payload for one complete asynchronous notification."""
    if len(body) < 8 or body[0] not in NOTIFICATION_IDS:
        return None
    payload_length = int.from_bytes(body[2:4], "big")
    expected_length = 4 + payload_length + 4
    if expected_length != len(body):
        return None
    return body[0], body[1], body[4:4 + payload_length]


def decode_response_en(body: bytes, command_id: int | None = None) -> str:
    notification = notification_fields(body)
    normal = normal_response_fields(body)
    if notification is not None and normal is None:
        response_id, status, payload = notification
        lines = [response_name(response_id), f"Status: {STATUS_EN.get(status, f'0x{status:02X}')}"]
        if response_id == 0x9C and len(payload) >= 3:
            temp_raw = int.from_bytes(payload[1:3], "big")
            lines.append(f"Thermometer {payload[0]} · {proto.tmp100_celsius(temp_raw):.2f} °C · raw 0x{temp_raw:04X}")
        elif response_id == 0x9D and len(payload) >= 13:
            mag_x, mag_y, mag_z = struct.unpack(">iii", payload[1:13])
            lines.append(f"Magnetometer {payload[0]} · X {mag_x} mG · Y {mag_y} mG · Z {mag_z} mG")
        elif payload:
            marker = "GOOD" if payload[1:3] == b"\x60\x0D" else "DEAD" if payload[1:3] == b"\xDE\xAD" else None
            lines.append(f"Payload: {proto.hex_bytes(payload)}" + (f" · {marker}" if marker else ""))
        return " | ".join(lines)
    if normal is None:
        return f"Invalid/incomplete frame ({len(body)} B): {proto.hex_bytes(body)}"
    response_id, sequence, status, payload = normal
    lines = [
        f"{response_name(response_id, command_id)} · ID 0x{response_id:02X} · SEQ {sequence}",
        f"Status: {STATUS_EN.get(status, f'0x{status:02X}')} · payload {len(payload)} B",
    ]
    if payload:
        if response_id == 0x8D:
            lines.append(payload.decode("ascii", errors="replace"))
        elif response_id == 0x94 and len(payload) >= 2:
            temp_raw = int.from_bytes(payload[:2], "big")
            lines.append(f"Temperature {proto.tmp100_celsius(temp_raw):.2f} °C · raw 0x{temp_raw:04X}")
        elif response_id == 0x96:
            lines.append(f"Magnetometer temperature {proto.mmc5603_celsius(payload[0]):.1f} °C · raw {payload[0]}")
        elif response_id == 0x97 and len(payload) >= 12:
            mag_x, mag_y, mag_z = struct.unpack(">iii", payload[:12])
            lines.append(f"Magnetic field · X {mag_x} mG · Y {mag_y} mG · Z {mag_z} mG")
        else:
            lines.append(f"Data: {proto.hex_bytes(payload)}")
    return " | ".join(lines)


def translated_choice(label: str) -> str:
    if label.startswith("Vzorek "):
        return label.replace("Vzorek", "Sample", 1)
    if label.startswith("Senzor "):
        return label.replace("Senzor", "Sensor", 1)
    return CHOICE_TRANSLATIONS.get(label, label)


def unit_limit_error(command_id: int, values: dict[str, Any], unit: int) -> str | None:
    if command_id not in UNIT_LIMITED_COMMANDS:
        return None
    unit_name, limit = UNIT_LIMITS[unit]
    for key, raw_value in values.items():
        if not key.startswith(("amplitude", "offset")):
            continue
        value = int(str(raw_value).strip(), 0)
        minimum = 0 if key.startswith("amplitude") else -limit
        if not minimum <= value <= limit:
            label = FIELD_LABELS.get(key, key)
            return f"{label}: allowed range is {minimum}…{limit} {unit_name} for the current input unit."
    return None


def magnetometer_rate_error(values: dict[str, Any]) -> str | None:
    magnetometer = int(str(values.get("magnetometer", "0")).strip(), 0)
    rate = int(str(values.get("rate", "0")).strip(), 0)
    if magnetometer in (0, 1) and rate > AMBIENT_MAGNETOMETER_MAX_PERIOD_S:
        return f"Ambient magnetometers use a period of 0…{AMBIENT_MAGNETOMETER_MAX_PERIOD_S} s."
    if magnetometer in (2, 3) and rate > COIL_MAGNETOMETER_MAX_FREQUENCY_HZ:
        return f"Coil magnetometers use a frequency of 0…{COIL_MAGNETOMETER_MAX_FREQUENCY_HZ} Hz."
    return None


class NanoRobotApp:
    def __init__(self, page: "ft.Page") -> None:
        self.page = page
        self.shutting_down = False
        self.serial_port: Any = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()
        self.rx_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.frame_parser = proto.FrameStreamParser()
        self.ascii_rx_buffer = bytearray()
        self.waiting_for_ready = False
        self.startup_deadline = 0.0
        self.startup_reports: dict[str, bool] = {}
        self.command_queue: list[QueuedCommand] = []
        self.awaiting_response: QueuedCommand | None = None
        self.response_deadline = 0.0
        self.response_fault = False
        self.sent_commands: dict[int, QueuedCommand] = {}
        self.tx_lock = threading.Lock()
        self.sequence_lock = threading.Lock()
        self.last_tx_time = 0.0
        self.sequence = 0
        self.log_lines: list[str] = []
        self.motor_active = False
        self.active_coil_outputs: set[int] = set()
        self.coil_state_unknown = False
        self.input_unit: int | None = None

        self.csv_lock = threading.Lock()
        self.csv_file: Any = None
        self.csv_writer: csv.DictWriter[str] | None = None
        self.csv_path: Path | None = None

        self.random_running = False
        self.random_generation = 0
        self.random_config: dict[str, Any] = {}
        self.random_started_at = 0.0
        self.random_ends_at = 0.0
        self.random_next_at = 0.0
        self.random_azimuth = 0.0
        self.random_elevation = 0.0

        self.current_command_id = 0x00
        self.command_fields: dict[str, Any] = {}
        self.current_nav_index = 0

        self._configure_page()
        self._build_shared_controls()
        self.manual_view = self._build_manual_view()
        self.random_view = self._build_random_view()
        self.uart_view = self._build_uart_view()
        self.diagnostics_view = self._build_diagnostics_view()
        self.views = [self.manual_view, self.random_view, self.uart_view, self.diagnostics_view]
        self._build_shell()
        self._select_command(0x00, update=False)
        self._refresh_ports(update=False)
        self.log("INFO", "NanoRobot Control Center ready · current firmware profile · input CRC disabled.")
        self.page.update()
        self.page.run_task(self._serial_poll_loop)

    def _configure_page(self) -> None:
        self.page.title = "NanoRobot Control Center"
        self.page.padding = 0
        self.page.bgcolor = BACKGROUND
        self.page.theme_mode = ft.ThemeMode.LIGHT
        self.page.theme = ft.Theme(color_scheme_seed=PRIMARY, use_material3=True, font_family="Segoe UI")
        self.page.window.width = 1500
        self.page.window.height = 960
        self.page.window.min_width = 1180
        self.page.window.min_height = 760
        self.page.window.bgcolor = BACKGROUND
        self.page.on_close = self._on_page_close

    @staticmethod
    def _input_style(**kwargs: Any) -> dict[str, Any]:
        base = {
            "dense": True,
            "filled": True,
            "fill_color": "#FBFAFE",
            "border_color": BORDER,
            "focused_border_color": PRIMARY,
            "border_radius": 12,
            "content_padding": 13,
        }
        base.update(kwargs)
        return base

    @staticmethod
    def _button_style(padding: int = 16) -> "ft.ButtonStyle":
        return ft.ButtonStyle(shape=ft.RoundedRectangleBorder(radius=12), padding=padding)

    def _card(
        self,
        content: "ft.Control",
        *,
        col: Any = 12,
        padding: int = 22,
        gradient: "ft.Gradient | None" = None,
        height: float | None = None,
    ) -> "ft.Container":
        return ft.Container(
            content=content,
            col=col,
            padding=padding,
            height=height,
            bgcolor=None if gradient else SURFACE,
            gradient=gradient,
            border=BORDER_ALL(1, BORDER),
            border_radius=20,
            shadow=ft.BoxShadow(blur_radius=24, color="#12000000", offset=(0, 8)),
        )

    @staticmethod
    def _eyebrow(text: str) -> "ft.Text":
        return ft.Text(
            text.upper(), size=11, weight=ft.FontWeight.W_600, color=PRIMARY,
            style=ft.TextStyle(letter_spacing=1.2),
        )

    @staticmethod
    def _title(text: str, size: int = 28) -> "ft.Text":
        return ft.Text(text, size=size, weight=ft.FontWeight.W_700, color=TEXT)

    def _status_chip(self, text: str, color: str, background: str, icon: Any) -> "ft.Container":
        return ft.Container(
            content=ft.Row(
                [ft.Icon(icon, size=15, color=color), ft.Text(text, size=12, weight=ft.FontWeight.W_600, color=color)],
                spacing=7,
                tight=True,
            ),
            bgcolor=background,
            border_radius=999,
            padding=PADDING_SYMMETRIC(horizontal=12, vertical=7),
        )

    def _build_shared_controls(self) -> None:
        self.port_dropdown = ft.Dropdown(
            label="Serial port",
            width=190,
            options=[],
            **self._input_style(),
        )
        self.connect_button = ft.FilledButton(
            "Connect",
            icon=ft.Icons.LINK_ROUNDED,
            bgcolor=PRIMARY,
            color=ft.Colors.WHITE,
            style=self._button_style(),
            on_click=self._toggle_connection,
        )
        self.connection_chip = self._status_chip("Disconnected", RED, RED_SOFT, ft.Icons.LINK_OFF_ROUNDED)
        self.random_header_chip = self._status_chip("Random ready", AMBER, AMBER_SOFT, ft.Icons.AUTO_AWESOME_ROUNDED)
        self.tx_delay_field = ft.TextField(
            label="TX delay",
            value="50",
            width=105,
            suffix=ft.Text("ms", size=10, color=MUTED),
            keyboard_type=ft.KeyboardType.NUMBER,
            tooltip="Minimum delay between UART transmissions; enter 0 to disable",
            **self._input_style(),
        )
        self.sequence_text = ft.Text("SEQ 0", size=12, color=MUTED, font_family=MONO)
        self.csv_log_status = ft.Text("CSV log starts automatically after connection", size=11, color=MUTED)
        self.recent_activity = ft.Column(spacing=7)
        # Internal status values are kept even though the separate Dashboard
        # page was removed. UART and Random Motion update them in one place.
        self.dashboard_connection_value = ft.Text("Offline")
        self.dashboard_random_value = ft.Text("Ready")
        self.coil_state_value = ft.Text("Outputs stopped", size=20, weight=ft.FontWeight.W_700, color=TEXT)
        self.dashboard_last_rx = ft.Text("No response")

        self.log_view = ft.ListView(expand=True, spacing=3, auto_scroll=True, padding=8)
        self.raw_hex = ft.TextField(
            label="Raw UART bytes (hex)",
            hint_text="7E 0D 00 00 00 00 00 00 00 7E",
            expand=True,
            text_style=ft.TextStyle(font_family=MONO, size=12),
            on_submit=self._send_raw_hex,
            **self._input_style(),
        )

    def _build_shell(self) -> None:
        brand = ft.Row(
            controls=[
                ft.Container(
                    width=46,
                    height=46,
                    border_radius=14,
                    gradient=ft.LinearGradient(colors=["#8B5CF6", "#6D28D9"], begin=ALIGN_TOP_LEFT, end=ALIGN_BOTTOM_RIGHT),
                    alignment=ALIGN_CENTER,
                    content=ft.Icon(ft.Icons.HUB_ROUNDED, color=ft.Colors.WHITE, size=25),
                ),
                ft.Column(
                    [
                        ft.Text("NanoRobot", size=20, weight=ft.FontWeight.W_700, color=TEXT),
                        ft.Text(
                            "CONTROL CENTER", size=9, weight=ft.FontWeight.W_600, color=MUTED,
                            style=ft.TextStyle(letter_spacing=1.5),
                        ),
                    ],
                    spacing=0,
                    tight=True,
                ),
            ],
            spacing=12,
            tight=True,
        )
        header = ft.Container(
            bgcolor=SURFACE,
            border=ft.Border(bottom=ft.BorderSide(1, BORDER)),
            padding=PADDING_SYMMETRIC(horizontal=24, vertical=14),
            content=ft.Row(
                controls=[
                    brand,
                    ft.Container(width=12),
                    self.port_dropdown,
                    ft.IconButton(
                        icon=ft.Icons.REFRESH_ROUNDED,
                        icon_color=PRIMARY,
                        tooltip="Refresh serial ports",
                        on_click=lambda e: self._refresh_ports(update=True),
                    ),
                    self.connect_button,
                    self.connection_chip,
                    self.random_header_chip,
                    ft.Container(expand=True),
                    self.tx_delay_field,
                    self.sequence_text,
                    ft.IconButton(
                        icon=ft.Icons.RESTART_ALT_ROUNDED,
                        icon_color=PRIMARY,
                        tooltip="Reset sequence number to 0",
                        on_click=self._reset_sequence,
                    ),
                    ft.OutlinedButton(
                        "Ping",
                        icon=ft.Icons.WIFI_TETHERING_ROUNDED,
                        style=self._button_style(14),
                        on_click=self._send_ping,
                    ),
                    ft.FilledButton(
                        "EMERGENCY STOP",
                        icon=ft.Icons.POWER_SETTINGS_NEW_ROUNDED,
                        bgcolor=RED,
                        color=ft.Colors.WHITE,
                        style=self._button_style(16),
                        on_click=self._emergency_stop,
                    ),
                ],
                spacing=10,
                vertical_alignment=ft.CrossAxisAlignment.CENTER,
            ),
        )

        self.navigation = ft.NavigationRail(
            selected_index=0,
            extended=True,
            min_extended_width=215,
            bgcolor=SURFACE,
            indicator_color=PRIMARY_SOFT,
            on_change=self._on_navigation_change,
            destinations=[
                ft.NavigationRailDestination(
                    icon=ft.Icons.TUNE_OUTLINED,
                    selected_icon=ft.Icons.TUNE_ROUNDED,
                    label="Manual Control",
                ),
                ft.NavigationRailDestination(
                    icon=ft.Icons.AUTO_AWESOME_OUTLINED,
                    selected_icon=ft.Icons.AUTO_AWESOME_ROUNDED,
                    label="Random Motion",
                ),
                ft.NavigationRailDestination(
                    icon=ft.Icons.SETTINGS_INPUT_COMPONENT_OUTLINED,
                    selected_icon=ft.Icons.SETTINGS_INPUT_COMPONENT_ROUNDED,
                    label="UART Commands",
                ),
                ft.NavigationRailDestination(
                    icon=ft.Icons.TERMINAL_OUTLINED,
                    selected_icon=ft.Icons.TERMINAL_ROUNDED,
                    label="Diagnostics",
                ),
            ],
            trailing=ft.Container(
                padding=12,
                content=ft.Column(
                    [
                        ft.Divider(color=BORDER),
                        ft.Text(f"v{APP_VERSION}", size=10, color=MUTED, text_align=ft.TextAlign.CENTER),
                        ft.Text(f"{proto.BAUDRATE} · 8N1", size=10, color=MUTED, text_align=ft.TextAlign.CENTER),
                    ],
                    horizontal_alignment=ft.CrossAxisAlignment.CENTER,
                    spacing=4,
                ),
            ),
        )
        self.content_host = ft.Container(
            expand=True,
            padding=24,
            content=self.manual_view,
            bgcolor=BACKGROUND,
        )
        shell = ft.Column(
            controls=[
                header,
                ft.Row(
                    controls=[
                        self.navigation,
                        ft.VerticalDivider(width=1, color=BORDER),
                        self.content_host,
                    ],
                    spacing=0,
                    expand=True,
                    vertical_alignment=ft.CrossAxisAlignment.STRETCH,
                ),
            ],
            spacing=0,
            expand=True,
        )
        self.page.add(shell)

    def _on_navigation_change(self, event: Any) -> None:
        self._show_view(int(event.control.selected_index))

    def _show_view(self, index: int) -> None:
        self.current_nav_index = index
        self.navigation.selected_index = index
        self.content_host.content = self.views[index]
        self.page.update()

    def _build_manual_view(self) -> "ft.Control":
        """Simple field, coil and motor controls for normal operation."""
        self.manual_unit = ft.Dropdown(
            label="Input units",
            value="2",
            options=[
                ft.DropdownOption(key="0", text="mV · millivolts"),
                ft.DropdownOption(key="1", text="mA · milliamps"),
                ft.DropdownOption(key="2", text="µT · magnetic field"),
            ],
            col={"sm": 12, "md": 4},
            **dropdown_event(self._update_manual_field_summary),
            **self._input_style(),
        )
        self.manual_azimuth = self._number_field("Azimuth", "0", "°", {"sm": 6, "md": 4})
        self.manual_elevation = self._number_field("Elevation", "0", "°", {"sm": 6, "md": 4})
        self.manual_amplitude = self._number_field("Amplitude", "100", "selected units", {"sm": 6, "md": 4})
        self.manual_frequency = self._number_field("Rotation frequency", "1", "Hz", {"sm": 6, "md": 4})
        self.manual_ramp = self._number_field("Ramp", "10", "× 0.1 s", {"sm": 6, "md": 4})
        self.manual_offset_x = self._number_field("Offset X", "0", "selected units", {"sm": 4, "md": 4})
        self.manual_offset_y = self._number_field("Offset Y", "0", "selected units", {"sm": 4, "md": 4})
        self.manual_offset_z = self._number_field("Offset Z", "0", "selected units", {"sm": 4, "md": 4})
        for control in (
            self.manual_azimuth, self.manual_elevation, self.manual_amplitude,
            self.manual_frequency, self.manual_ramp, self.manual_offset_x,
            self.manual_offset_y, self.manual_offset_z,
        ):
            control.on_change = self._update_manual_field_summary

        self.manual_enable_outputs = ft.Switch(
            label="Enable all coil outputs after applying the field",
            value=False,
            active_color=PRIMARY,
        )
        self.manual_field_summary = ft.Text(
            "0° azimuth · 0° elevation · 100 µT · 1 Hz",
            size=14,
            color=TEXT,
            weight=ft.FontWeight.W_600,
        )
        self.manual_field_status = ft.Text("Ready to apply", size=12, color=MUTED)

        field_card = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Container(
                                width=48,
                                height=48,
                                border_radius=15,
                                bgcolor=PRIMARY_SOFT,
                                alignment=ALIGN_CENTER,
                                content=ft.Icon(ft.Icons.EXPLORE_ROUNDED, color=PRIMARY, size=26),
                            ),
                            ft.Column(
                                [
                                    ft.Text("Magnetic field", size=20, weight=ft.FontWeight.W_700, color=TEXT),
                                    ft.Text("Simple rotating-field control · CMD 0x00 + 0x06", size=12, color=MUTED),
                                ],
                                spacing=1,
                                expand=True,
                            ),
                        ],
                        spacing=13,
                    ),
                    ft.Divider(color=BORDER),
                    self._eyebrow("Direction and units"),
                    ft.ResponsiveRow(
                        [self.manual_unit, self.manual_azimuth, self.manual_elevation],
                        spacing=10,
                        run_spacing=10,
                    ),
                    self._eyebrow("Field motion"),
                    ft.ResponsiveRow(
                        [self.manual_amplitude, self.manual_frequency, self.manual_ramp],
                        spacing=10,
                        run_spacing=10,
                    ),
                    self._eyebrow("Offsets"),
                    ft.ResponsiveRow(
                        [self.manual_offset_x, self.manual_offset_y, self.manual_offset_z],
                        spacing=10,
                        run_spacing=10,
                    ),
                    self.manual_enable_outputs,
                    ft.Row(
                        [
                            ft.FilledButton(
                                "APPLY FIELD",
                                icon=ft.Icons.SEND_ROUNDED,
                                bgcolor=PRIMARY,
                                color=ft.Colors.WHITE,
                                style=self._button_style(16),
                                on_click=self._apply_manual_field,
                            ),
                            ft.OutlinedButton(
                                "ZERO FIELD",
                                icon=ft.Icons.RESTART_ALT_ROUNDED,
                                style=self._button_style(16),
                                on_click=self._zero_manual_field,
                            ),
                        ],
                        spacing=10,
                        wrap=True,
                    ),
                ],
                spacing=12,
            ),
            col={"sm": 12, "lg": 8},
        )

        output_card = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Container(
                                width=48,
                                height=48,
                                border_radius=15,
                                bgcolor="#EDE9FE",
                                alignment=ALIGN_CENTER,
                                content=ft.Icon(ft.Icons.WAVES_ROUNDED, color=PRIMARY, size=26),
                            ),
                            ft.Column(
                                [
                                    ft.Text("Coil outputs", size=20, weight=ft.FontWeight.W_700, color=TEXT),
                                    ft.Text("Global output control · CMD 0x07", size=12, color=MUTED),
                                ],
                                spacing=1,
                            ),
                        ],
                        spacing=13,
                    ),
                    ft.Divider(color=BORDER),
                    self._eyebrow("Current field"),
                    self.manual_field_summary,
                    self.manual_field_status,
                    self._eyebrow("Output state"),
                    self.coil_state_value,
                    ft.FilledButton(
                        "RUN ALL COILS",
                        icon=ft.Icons.PLAY_ARROW_ROUNDED,
                        bgcolor=PRIMARY,
                        color=ft.Colors.WHITE,
                        style=self._button_style(16),
                        on_click=self._confirm_coils_on,
                    ),
                    ft.OutlinedButton(
                        "STOP ALL COILS",
                        icon=ft.Icons.STOP_ROUNDED,
                        style=self._button_style(16),
                        on_click=lambda e: self._set_all_coils(False),
                    ),
                    ft.Text(
                        "APPLY FIELD configures the field. Coil power can be controlled independently.",
                        size=11,
                        color=MUTED,
                    ),
                ],
                spacing=12,
                horizontal_alignment=ft.CrossAxisAlignment.STRETCH,
            ),
            col={"sm": 12, "lg": 4},
        )

        self.manual_motor_select = ft.Dropdown(
            label="Motor",
            value="0",
            options=[
                ft.DropdownOption(key="0", text="Exchange motor"),
                ft.DropdownOption(key="1", text="Focus motor"),
            ],
            col={"sm": 12, "md": 4},
            **self._input_style(),
        )
        self.manual_motor_steps = ft.TextField(
            label="Steps",
            value="100",
            keyboard_type=ft.KeyboardType.NUMBER,
            col={"sm": 12, "md": 4},
            **self._input_style(),
        )
        self.manual_motor_sample = ft.Dropdown(
            label="Sample position",
            value="0",
            options=[ft.DropdownOption(key=str(index), text=f"Sample {index:02d}") for index in range(20)],
            col={"sm": 12, "md": 4},
            **self._input_style(),
        )
        self.manual_motor_direction = ft.SegmentedButton(
            segments=[
                ft.Segment(value="0", icon=ft.Icons.ARROW_UPWARD_ROUNDED, label="Forward / up"),
                ft.Segment(value="1", icon=ft.Icons.ARROW_DOWNWARD_ROUNDED, label="Reverse / down"),
            ],
            selected=["0"],
        )

        motor_card = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Container(
                                width=48,
                                height=48,
                                border_radius=15,
                                bgcolor=PRIMARY_SOFT,
                                alignment=ALIGN_CENTER,
                                content=ft.Icon(ft.Icons.PRECISION_MANUFACTURING_ROUNDED, color=PRIMARY, size=26),
                            ),
                            ft.Column(
                                [
                                    ft.Text("Motor positioning", size=20, weight=ft.FontWeight.W_700, color=TEXT),
                                    ft.Text("Reference, move and sample positions · CMD 0x0E–0x11", size=12, color=MUTED),
                                ],
                                spacing=1,
                                expand=True,
                            ),
                        ],
                        spacing=13,
                    ),
                    ft.Divider(color=BORDER),
                    ft.ResponsiveRow(
                        [self.manual_motor_select, self.manual_motor_steps, self.manual_motor_sample],
                        spacing=10,
                        run_spacing=10,
                    ),
                    ft.Container(
                        content=self.manual_motor_direction,
                        bgcolor="#FAF7FF",
                        border=BORDER_ALL(1, BORDER),
                        border_radius=14,
                        padding=10,
                    ),
                    ft.Row(
                        [
                            ft.Text("Quick steps", size=11, color=MUTED),
                            ft.TextButton("10", on_click=lambda e: self._set_manual_motor_steps(10)),
                            ft.TextButton("100", on_click=lambda e: self._set_manual_motor_steps(100)),
                            ft.TextButton("1000", on_click=lambda e: self._set_manual_motor_steps(1000)),
                            ft.OutlinedButton(
                                "Reference",
                                icon=ft.Icons.HOME_ROUNDED,
                                style=self._button_style(13),
                                on_click=self._confirm_manual_motor_reference,
                            ),
                            ft.FilledButton(
                                "MOVE",
                                icon=ft.Icons.DOUBLE_ARROW_ROUNDED,
                                bgcolor=PRIMARY,
                                color=ft.Colors.WHITE,
                                style=self._button_style(13),
                                on_click=self._move_manual_motor,
                            ),
                            ft.OutlinedButton(
                                "Go to sample",
                                icon=ft.Icons.PLACE_ROUNDED,
                                style=self._button_style(13),
                                on_click=self._manual_go_to_sample,
                            ),
                            ft.OutlinedButton(
                                "Save sample",
                                icon=ft.Icons.SAVE_ROUNDED,
                                style=self._button_style(13),
                                on_click=self._confirm_manual_save_sample,
                            ),
                        ],
                        spacing=7,
                        wrap=True,
                    ),
                ],
                spacing=12,
            ),
        )

        return ft.Column(
            controls=[
                self._eyebrow("Direct control"),
                self._title("Manual Control"),
                ft.Text(
                    "Set the rotating field, coil outputs and motor position from one simple screen.",
                    color=MUTED,
                    size=13,
                ),
                ft.ResponsiveRow([field_card, output_card], spacing=14, run_spacing=14),
                motor_card,
            ],
            spacing=12,
            scroll=ft.ScrollMode.AUTO,
            expand=True,
        )

    def _metric_card(
        self,
        title: str,
        value_control: "ft.Control",
        subtitle: str,
        icon: Any,
        accent: str,
    ) -> "ft.Container":
        return self._card(
            ft.Row(
                controls=[
                    ft.Container(
                        width=48,
                        height=48,
                        border_radius=15,
                        bgcolor=f"{accent}18",
                        alignment=ALIGN_CENTER,
                        content=ft.Icon(icon, color=accent, size=25),
                    ),
                    ft.Column(
                        [
                            ft.Text(
                                title.upper(), size=10, weight=ft.FontWeight.W_600, color=MUTED,
                                style=ft.TextStyle(letter_spacing=1.0),
                            ),
                            value_control,
                            ft.Text(subtitle, size=11, color=MUTED),
                        ],
                        spacing=1,
                        expand=True,
                    ),
                ],
                spacing=14,
            ),
            col={"sm": 12, "md": 6, "lg": 3},
            padding=18,
        )

    def _build_dashboard_view(self) -> "ft.Control":
        self.dashboard_connection_value = ft.Text("Offline", size=20, weight=ft.FontWeight.W_700, color=TEXT)
        self.dashboard_random_value = ft.Text("Ready", size=20, weight=ft.FontWeight.W_700, color=TEXT)
        self.coil_state_value = ft.Text("Outputs stopped", size=20, weight=ft.FontWeight.W_700, color=TEXT)
        self.dashboard_last_rx = ft.Text("No response", size=20, weight=ft.FontWeight.W_700, color=TEXT, no_wrap=True)

        metrics = ft.ResponsiveRow(
            controls=[
                self._metric_card("Connection", self.dashboard_connection_value, f"UART4 · {proto.BAUDRATE} baud", ft.Icons.USB_ROUNDED, PRIMARY),
                self._metric_card("Coils", self.coil_state_value, "Global output state", ft.Icons.WAVES_ROUNDED, BLUE),
                self._metric_card("Random Motion", self.dashboard_random_value, "CMD 0x06 automation", ft.Icons.AUTO_AWESOME_ROUNDED, AMBER),
                self._metric_card("Last response", self.dashboard_last_rx, "Decoded UART activity", ft.Icons.MARK_CHAT_READ_ROUNDED, GREEN),
            ],
            spacing=14,
            run_spacing=14,
        )

        coils_tile = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Container(
                                width=46,
                                height=46,
                                border_radius=14,
                                bgcolor="#EDE9FE",
                                alignment=ALIGN_CENTER,
                                content=ft.Icon(ft.Icons.WAVES_ROUNDED, color=PRIMARY, size=25),
                            ),
                            ft.Column(
                                [ft.Text("Coil outputs", size=18, weight=ft.FontWeight.W_700, color=TEXT),
                                 ft.Text("Fast global control", size=12, color=MUTED)],
                                spacing=1,
                                expand=True,
                            ),
                        ]
                    ),
                    ft.Text(
                        "Configure the field first, then enable all outputs. STOP is always available in the top bar.",
                        size=12,
                        color=MUTED,
                    ),
                    ft.Row(
                        [
                            ft.FilledButton(
                                "RUN ALL",
                                icon=ft.Icons.PLAY_ARROW_ROUNDED,
                                bgcolor=PRIMARY,
                                color=ft.Colors.WHITE,
                                style=self._button_style(),
                                on_click=self._confirm_coils_on,
                                expand=True,
                            ),
                            ft.OutlinedButton(
                                "STOP ALL",
                                icon=ft.Icons.STOP_ROUNDED,
                                style=self._button_style(),
                                on_click=lambda e: self._set_all_coils(False),
                                expand=True,
                            ),
                        ],
                        spacing=10,
                    ),
                ],
                spacing=16,
            ),
            col={"sm": 12, "lg": 4},
            height=250,
        )

        self.motor_select = ft.Dropdown(
            label="Motor",
            value="0",
            options=[
                ft.DropdownOption(key="0", text="Exchange motor"),
                ft.DropdownOption(key="1", text="Focus motor"),
            ],
            expand=True,
            **self._input_style(),
        )
        self.motor_direction = ft.SegmentedButton(
            segments=[
                ft.Segment(value="0", icon=ft.Icons.ARROW_UPWARD_ROUNDED, label="Forward"),
                ft.Segment(value="1", icon=ft.Icons.ARROW_DOWNWARD_ROUNDED, label="Reverse"),
            ],
            selected=["0"],
            expand=True,
        )
        self.motor_steps = ft.TextField(
            label="Steps",
            value="100",
            keyboard_type=ft.KeyboardType.NUMBER,
            expand=True,
            **self._input_style(),
        )
        motor_tile = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Container(
                                width=46,
                                height=46,
                                border_radius=14,
                                bgcolor="#EDE9FE",
                                alignment=ALIGN_CENTER,
                                content=ft.Icon(ft.Icons.PRECISION_MANUFACTURING_ROUNDED, color=PRIMARY, size=25),
                            ),
                            ft.Column(
                                [ft.Text("Motor movement", size=18, weight=ft.FontWeight.W_700, color=TEXT),
                                 ft.Text("CMD 0x0F · relative move", size=12, color=MUTED)],
                                spacing=1,
                                expand=True,
                            ),
                        ]
                    ),
                    ft.Row([self.motor_select, self.motor_steps], spacing=10),
                    self.motor_direction,
                    ft.Row(
                        [
                            ft.TextButton("10", on_click=lambda e: self._set_motor_steps(10)),
                            ft.TextButton("100", on_click=lambda e: self._set_motor_steps(100)),
                            ft.TextButton("1000", on_click=lambda e: self._set_motor_steps(1000)),
                            ft.Container(expand=True),
                            ft.OutlinedButton("Reference", icon=ft.Icons.HOME_ROUNDED, on_click=self._confirm_motor_reference),
                            ft.FilledButton(
                                "MOVE",
                                icon=ft.Icons.DOUBLE_ARROW_ROUNDED,
                                bgcolor=PRIMARY,
                                color=ft.Colors.WHITE,
                                style=self._button_style(),
                                on_click=self._move_motor,
                            ),
                        ],
                        spacing=7,
                    ),
                ],
                spacing=12,
            ),
            col={"sm": 12, "lg": 8},
            height=250,
        )

        random_tile = self._card(
            ft.Column(
                controls=[
                    self._eyebrow("Automation"),
                    ft.Text("Random Motion", size=24, weight=ft.FontWeight.W_700, color=TEXT),
                    ft.Text(
                        "Generate fully random or smooth rotating-field trajectories with live progress and a safety stop.",
                        size=13,
                        color=MUTED,
                    ),
                    ft.Container(expand=True),
                    ft.FilledButton(
                        "OPEN RANDOM MOTION",
                        icon=ft.Icons.AUTO_AWESOME_ROUNDED,
                        bgcolor=PRIMARY,
                        color=ft.Colors.WHITE,
                        style=self._button_style(),
                        on_click=lambda e: self._show_view(1),
                    ),
                ],
                spacing=9,
                expand=True,
            ),
            col={"sm": 12, "lg": 5},
            height=210,
            gradient=ft.LinearGradient(colors=["#FFFFFF", "#F3E8FF"], begin=ALIGN_TOP_LEFT, end=ALIGN_BOTTOM_RIGHT),
        )
        activity_tile = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Text("Recent activity", size=18, weight=ft.FontWeight.W_700, color=TEXT),
                            ft.Container(expand=True),
                            ft.TextButton("Open diagnostics", on_click=lambda e: self._show_view(3)),
                        ]
                    ),
                    ft.Divider(color=BORDER),
                    self.recent_activity,
                ],
                spacing=8,
            ),
            col={"sm": 12, "lg": 7},
            height=210,
        )

        return ft.Column(
            controls=[
                self._eyebrow("System overview"),
                self._title("Nanorobot dashboard"),
                ft.Text("Fast access to the controls used most often during bench testing.", color=MUTED, size=13),
                ft.Container(height=4),
                metrics,
                ft.ResponsiveRow([coils_tile, motor_tile], spacing=14, run_spacing=14),
                ft.ResponsiveRow([random_tile, activity_tile], spacing=14, run_spacing=14),
            ],
            spacing=14,
            scroll=ft.ScrollMode.AUTO,
            expand=True,
        )

    def _number_field(self, label: str, value: str, suffix: str = "", col: Any = 6) -> "ft.TextField":
        return ft.TextField(
            label=label,
            value=value,
            suffix=ft.Text(suffix, size=11, color=MUTED) if suffix else None,
            keyboard_type=ft.KeyboardType.NUMBER,
            col=col,
            **self._input_style(),
        )

    def _build_random_view(self) -> "ft.Control":
        self.random_mode = ft.Dropdown(
            label="Randomization mode",
            value="smooth",
            options=[
                ft.DropdownOption(key="random", text="Fully random"),
                ft.DropdownOption(key="smooth", text="Smooth steps"),
            ],
            col={"sm": 12, "md": 4},
            **dropdown_event(self._random_mode_changed),
            **self._input_style(),
        )
        self.random_duration = self._number_field("Duration", "60", "s", {"sm": 6, "md": 4})
        self.random_interval = self._number_field("Change interval", "1.0", "s", {"sm": 6, "md": 4})
        self.random_az_min = self._number_field("Azimuth min", "0", "°", 6)
        self.random_az_max = self._number_field("Azimuth max", "360", "°", 6)
        self.random_el_min = self._number_field("Elevation min", "-90", "°", 6)
        self.random_el_max = self._number_field("Elevation max", "90", "°", 6)
        self.random_step_az = self._number_field("Maximum azimuth step", "20", "°", 6)
        self.random_step_el = self._number_field("Maximum elevation step", "10", "°", 6)
        self.random_amplitude = self._number_field("Amplitude", "100", "µT", 4)
        self.random_frequency = self._number_field("Rotation frequency", "1", "Hz", 4)
        self.random_ramp = self._number_field("Ramp", "10", "× 0.1 s", 4)
        self.random_enable_outputs = ft.Switch(
            label="Enable all coil outputs on start",
            value=True,
            active_color=PRIMARY,
        )
        self.random_power_off = ft.Switch(
            label="Stop coil outputs when finished",
            value=True,
            active_color=PRIMARY,
        )
        self.random_start_button = ft.FilledButton(
            "START RANDOM MOTION",
            icon=ft.Icons.PLAY_ARROW_ROUNDED,
            bgcolor=PRIMARY,
            color=ft.Colors.WHITE,
            style=self._button_style(18),
            on_click=self._start_random_motion,
        )
        self.random_pause_button = ft.OutlinedButton(
            "STOP CHANGES",
            icon=ft.Icons.PAUSE_ROUNDED,
            style=self._button_style(18),
            disabled=True,
            on_click=self._stop_random_changes,
        )

        # Manual motor controls live directly on the Random Motion page so the
        # operator can adjust the sample/focus position without changing tabs.
        self.motor_select = ft.Dropdown(
            label="Motor",
            value="0",
            options=[
                ft.DropdownOption(key="0", text="Exchange motor"),
                ft.DropdownOption(key="1", text="Focus motor"),
            ],
            col={"sm": 12, "md": 4},
            **self._input_style(),
        )
        self.motor_steps = ft.TextField(
            label="Steps",
            value="100",
            keyboard_type=ft.KeyboardType.NUMBER,
            col={"sm": 12, "md": 4},
            **self._input_style(),
        )
        self.motor_sample = ft.Dropdown(
            label="Sample position",
            value="0",
            options=[ft.DropdownOption(key=str(index), text=f"Sample {index:02d}") for index in range(20)],
            col={"sm": 12, "md": 4},
            **self._input_style(),
        )
        self.motor_direction = ft.SegmentedButton(
            segments=[
                ft.Segment(value="0", icon=ft.Icons.ARROW_UPWARD_ROUNDED, label="Forward / up"),
                ft.Segment(value="1", icon=ft.Icons.ARROW_DOWNWARD_ROUNDED, label="Reverse / down"),
            ],
            selected=["0"],
        )

        setup_card = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Container(
                                width=48,
                                height=48,
                                border_radius=15,
                                bgcolor=PRIMARY_SOFT,
                                alignment=ALIGN_CENTER,
                                content=ft.Icon(ft.Icons.TUNE_ROUNDED, color=PRIMARY, size=25),
                            ),
                            ft.Column(
                                [
                                    ft.Text("Motion setup", size=20, weight=ft.FontWeight.W_700, color=TEXT),
                                    ft.Text("Rotating magnetic field · CMD 0x06", size=12, color=MUTED),
                                ],
                                spacing=1,
                            ),
                        ],
                        spacing=13,
                    ),
                    ft.Divider(color=BORDER),
                    self._eyebrow("Timeline"),
                    ft.ResponsiveRow([self.random_mode, self.random_duration, self.random_interval], spacing=10, run_spacing=10),
                    self._eyebrow("Motion envelope"),
                    ft.ResponsiveRow(
                        [self.random_az_min, self.random_az_max, self.random_el_min, self.random_el_max],
                        spacing=10,
                        run_spacing=10,
                    ),
                    ft.ResponsiveRow([self.random_step_az, self.random_step_el], spacing=10, run_spacing=10),
                    self._eyebrow("Field parameters"),
                    ft.ResponsiveRow(
                        [self.random_amplitude, self.random_frequency, self.random_ramp],
                        spacing=10,
                        run_spacing=10,
                    ),
                    ft.Container(
                        bgcolor="#FAF7FF",
                        border=BORDER_ALL(1, BORDER),
                        border_radius=14,
                        padding=12,
                        content=ft.ResponsiveRow(
                            [
                                ft.Container(content=self.random_enable_outputs, col={"sm": 12, "lg": 6}),
                                ft.Container(content=self.random_power_off, col={"sm": 12, "lg": 6}),
                            ],
                            spacing=4,
                            run_spacing=4,
                        ),
                    ),
                    ft.Row([self.random_start_button, self.random_pause_button], spacing=10, wrap=True),
                    ft.Container(
                        bgcolor="#FAF7FF",
                        border=BORDER_ALL(1, BORDER),
                        border_radius=14,
                        padding=12,
                        content=ft.Row(
                            [
                                ft.Icon(ft.Icons.WAVES_ROUNDED, color=PRIMARY, size=21),
                                ft.Text("Manual coil outputs", size=12, weight=ft.FontWeight.W_600, color=TEXT),
                                ft.FilledButton(
                                    "RUN ALL",
                                    icon=ft.Icons.PLAY_ARROW_ROUNDED,
                                    bgcolor=PRIMARY,
                                    color=ft.Colors.WHITE,
                                    style=self._button_style(12),
                                    on_click=self._confirm_coils_on,
                                ),
                                ft.OutlinedButton(
                                    "STOP ALL",
                                    icon=ft.Icons.STOP_ROUNDED,
                                    style=self._button_style(12),
                                    on_click=lambda e: self._set_all_coils(False),
                                ),
                            ],
                            spacing=8,
                            wrap=True,
                        ),
                    ),
                ],
                spacing=12,
            ),
            col={"sm": 12, "lg": 7},
        )

        self.random_status_text = ft.Text("READY", size=12, weight=ft.FontWeight.W_700, color=AMBER)
        self.random_status_box = ft.Container(
            content=self.random_status_text,
            bgcolor=AMBER_SOFT,
            border_radius=999,
            padding=PADDING_SYMMETRIC(horizontal=12, vertical=7),
        )
        self.random_az_text = ft.Text("—", size=28, weight=ft.FontWeight.W_700, color=TEXT)
        self.random_el_text = ft.Text("—", size=28, weight=ft.FontWeight.W_700, color=TEXT)
        self.random_remaining_text = ft.Text("01:00.0", size=28, weight=ft.FontWeight.W_700, color=TEXT)
        self.random_next_text = ft.Text("—", size=28, weight=ft.FontWeight.W_700, color=TEXT)
        self.random_progress = ft.ProgressBar(value=0, color=PRIMARY, bgcolor="#EDE9FE", height=8, border_radius=8)
        self.compass_arrow = ft.Container(
            width=150,
            height=150,
            alignment=ALIGN_CENTER,
            content=ft.Icon(ft.Icons.NORTH_ROUNDED, size=94, color=PRIMARY),
            rotate=ft.Rotate(angle=0),
            animate_rotation=300,
        )
        compass = ft.Container(
            width=300,
            height=300,
            border_radius=150,
            border=BORDER_ALL(2, "#D8CCED"),
            gradient=ft.RadialGradient(colors=["#FFFFFF", "#F3E8FF"]),
            alignment=ALIGN_CENTER,
            content=ft.Stack(
                controls=[
                    ft.Text("0°", top=10, left=139, size=11, color=MUTED),
                    ft.Text("90°", top=142, right=9, size=11, color=MUTED),
                    ft.Text("180°", bottom=10, left=130, size=11, color=MUTED),
                    ft.Text("270°", top=142, left=7, size=11, color=MUTED),
                    ft.Container(width=1, height=248, bgcolor="#E1D8F1", left=149, top=26),
                    ft.Container(width=248, height=1, bgcolor="#E1D8F1", left=26, top=149),
                    ft.Container(content=self.compass_arrow, left=75, top=75),
                    ft.Container(width=12, height=12, border_radius=6, bgcolor=PRIMARY_DARK, left=144, top=144),
                ]
            ),
        )

        def metric(label: str, control: "ft.Control") -> "ft.Container":
            return ft.Container(
                content=ft.Column([ft.Text(label, size=10, color=MUTED, weight=ft.FontWeight.W_600), control], spacing=1),
                col=6,
            )

        monitor_card = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Column(
                                [ft.Text("Live field", size=20, weight=ft.FontWeight.W_700, color=TEXT),
                                 ft.Text("Current generated orientation", size=12, color=MUTED)],
                                spacing=1,
                            ),
                            ft.Container(expand=True),
                            self.random_status_box,
                        ]
                    ),
                    ft.ResponsiveRow(
                        [
                            metric("AZIMUTH", self.random_az_text),
                            metric("ELEVATION", self.random_el_text),
                            metric("REMAINING", self.random_remaining_text),
                            metric("NEXT CHANGE", self.random_next_text),
                        ],
                        spacing=10,
                        run_spacing=8,
                    ),
                    ft.Container(content=compass, alignment=ALIGN_CENTER, padding=8),
                    self.random_progress,
                    ft.FilledButton(
                        "STOP RANDOM + COILS OFF",
                        icon=ft.Icons.POWER_SETTINGS_NEW_ROUNDED,
                        bgcolor=RED,
                        color=ft.Colors.WHITE,
                        style=self._button_style(18),
                        on_click=self._random_emergency_stop,
                    ),
                    ft.Text(
                        "STOP CHANGES leaves the last field active. The red button also sends 0x07 / ALL / STOP.",
                        size=11,
                        color=MUTED,
                        text_align=ft.TextAlign.CENTER,
                    ),
                ],
                spacing=14,
                horizontal_alignment=ft.CrossAxisAlignment.STRETCH,
            ),
            col={"sm": 12, "lg": 5},
        )

        motor_card = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Container(
                                width=48,
                                height=48,
                                border_radius=15,
                                bgcolor=PRIMARY_SOFT,
                                alignment=ALIGN_CENTER,
                                content=ft.Icon(ft.Icons.PRECISION_MANUFACTURING_ROUNDED, color=PRIMARY, size=26),
                            ),
                            ft.Column(
                                [
                                    ft.Text("Manual motor control", size=20, weight=ft.FontWeight.W_700, color=TEXT),
                                    ft.Text("Reference, relative movement and sample positions · CMD 0x0E–0x11", size=12, color=MUTED),
                                ],
                                spacing=1,
                                expand=True,
                            ),
                            ft.Container(
                                content=ft.Text("MOTOR", size=10, color=PRIMARY, weight=ft.FontWeight.W_700),
                                bgcolor=PRIMARY_SOFT,
                                border_radius=999,
                                padding=PADDING_SYMMETRIC(horizontal=11, vertical=6),
                            ),
                        ],
                        spacing=13,
                    ),
                    ft.Divider(color=BORDER),
                    ft.ResponsiveRow(
                        [self.motor_select, self.motor_steps, self.motor_sample],
                        spacing=10,
                        run_spacing=10,
                    ),
                    ft.Container(
                        content=self.motor_direction,
                        bgcolor="#FAF7FF",
                        border=BORDER_ALL(1, BORDER),
                        border_radius=14,
                        padding=10,
                    ),
                    ft.Row(
                        [
                            ft.Text("Quick steps", size=11, color=MUTED),
                            ft.TextButton("10", on_click=lambda e: self._set_motor_steps(10)),
                            ft.TextButton("100", on_click=lambda e: self._set_motor_steps(100)),
                            ft.TextButton("1000", on_click=lambda e: self._set_motor_steps(1000)),
                            ft.OutlinedButton(
                                "Reference",
                                icon=ft.Icons.HOME_ROUNDED,
                                style=self._button_style(13),
                                on_click=self._confirm_motor_reference,
                            ),
                            ft.FilledButton(
                                "MOVE",
                                icon=ft.Icons.DOUBLE_ARROW_ROUNDED,
                                bgcolor=PRIMARY,
                                color=ft.Colors.WHITE,
                                style=self._button_style(13),
                                on_click=self._move_motor,
                            ),
                            ft.OutlinedButton(
                                "Go to sample",
                                icon=ft.Icons.PLACE_ROUNDED,
                                style=self._button_style(13),
                                on_click=self._go_to_sample,
                            ),
                            ft.OutlinedButton(
                                "Save sample",
                                icon=ft.Icons.SAVE_ROUNDED,
                                style=self._button_style(13),
                                on_click=self._confirm_save_sample,
                            ),
                        ],
                        spacing=7,
                        wrap=True,
                    ),
                    ft.Text(
                        "Motor commands can be used while Random Motion is active. Referencing moves toward a mechanical stop.",
                        size=11,
                        color=MUTED,
                    ),
                ],
                spacing=12,
            ),
        )

        return ft.Column(
            controls=[
                self._eyebrow("Field automation"),
                self._title("Random Motion"),
                ft.Text("Generate controlled random trajectories inside a safe angular envelope.", color=MUTED, size=13),
                ft.ResponsiveRow([setup_card, monitor_card], spacing=14, run_spacing=14),
                motor_card,
            ],
            spacing=12,
            scroll=ft.ScrollMode.AUTO,
            expand=True,
        )

    def _build_uart_view(self) -> "ft.Control":
        self.command_category = ft.Dropdown(
            label="Category",
            value="All",
            options=[ft.DropdownOption(key=value, text=value) for value in (
                "All", "Coils", "Calibration", "Motors", "Peripherals", "Sensors", "Communication"
            )],
            **dropdown_event(self._filter_command_list),
            **self._input_style(),
        )
        self.command_list = ft.ListView(expand=True, spacing=6, padding=2)
        self.command_title_text = ft.Text("Input units", size=23, weight=ft.FontWeight.W_700, color=TEXT)
        self.command_id_text = ft.Text("0x00", size=12, color=PRIMARY, weight=ft.FontWeight.W_700, font_family=MONO)
        self.command_summary_text = ft.Text("", size=12, color=MUTED)
        self.command_warning = ft.Container(
            visible=False,
            bgcolor=AMBER_SOFT,
            border_radius=12,
            padding=12,
            content=ft.Row(
                [ft.Icon(ft.Icons.WARNING_AMBER_ROUNDED, color=AMBER), ft.Text("", size=12, color="#92400E", expand=True)],
            ),
        )
        self.command_form_host = ft.Container()
        self.command_preview = ft.Text(
            "",
            size=11,
            color="#4C1D95",
            font_family=MONO,
            selectable=True,
        )
        self.command_preview_host = ft.Container(
            content=self.command_preview,
            bgcolor="#FAF7FF",
            border=BORDER_ALL(1, BORDER),
            border_radius=14,
            padding=14,
            expand=True,
        )
        self.command_send_button = ft.FilledButton(
            "SEND COMMAND",
            icon=ft.Icons.SEND_ROUNDED,
            bgcolor=PRIMARY,
            color=ft.Colors.WHITE,
            style=self._button_style(18),
            on_click=self._send_selected_command,
        )

        selector = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Text("Command library", size=18, weight=ft.FontWeight.W_700, color=TEXT),
                            ft.Container(expand=True),
                            ft.Container(
                                content=ft.Text("0x00–0x18", size=10, color=PRIMARY, font_family=MONO),
                                bgcolor=PRIMARY_SOFT,
                                border_radius=999,
                                padding=PADDING_SYMMETRIC(horizontal=9, vertical=5),
                            ),
                        ]
                    ),
                    self.command_category,
                    self.command_list,
                ],
                spacing=10,
                expand=True,
            ),
            col={"sm": 12, "lg": 3},
            height=670,
            padding=16,
        )
        editor = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Column([self.command_id_text, self.command_title_text], spacing=0, expand=True),
                            ft.Container(
                                content=ft.Text("CURRENT FIRMWARE", size=10, color=PRIMARY, weight=ft.FontWeight.W_700),
                                bgcolor=PRIMARY_SOFT,
                                border_radius=999,
                                padding=PADDING_SYMMETRIC(horizontal=11, vertical=6),
                            ),
                        ]
                    ),
                    self.command_summary_text,
                    self.command_warning,
                    ft.Divider(color=BORDER),
                    self.command_form_host,
                    ft.Container(expand=True),
                    ft.Row(
                        [
                            ft.Text("CRC bytes: 00 00 00 00", size=11, color=MUTED, font_family=MONO),
                            ft.Container(expand=True),
                            self.command_send_button,
                        ]
                    ),
                ],
                spacing=12,
                expand=True,
            ),
            col={"sm": 12, "lg": 5},
            height=670,
        )
        preview = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Text("Frame preview", size=18, weight=ft.FontWeight.W_700, color=TEXT),
                            ft.Container(expand=True),
                            ft.Icon(ft.Icons.DATA_OBJECT_ROUNDED, color=PRIMARY),
                        ]
                    ),
                    ft.Text("Big-endian payload length and structures", size=11, color=MUTED),
                    self.command_preview_host,
                ],
                spacing=10,
                expand=True,
            ),
            col={"sm": 12, "lg": 4},
            height=670,
        )
        self._populate_command_list("All")
        return ft.Column(
            controls=[
                self._eyebrow("Advanced control"),
                self._title("UART Commands"),
                ft.Text("Every firmware command in one validated, human-readable editor.", size=13, color=MUTED),
                ft.ResponsiveRow([selector, editor, preview], spacing=14, run_spacing=14),
            ],
            spacing=12,
            scroll=ft.ScrollMode.AUTO,
            expand=True,
        )

    def _build_diagnostics_view(self) -> "ft.Control":
        raw_card = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Container(
                                width=44,
                                height=44,
                                border_radius=14,
                                bgcolor=PRIMARY_SOFT,
                                alignment=ALIGN_CENTER,
                                content=ft.Icon(ft.Icons.CODE_ROUNDED, color=PRIMARY),
                            ),
                            ft.Column(
                                [ft.Text("Raw transmitter", size=18, weight=ft.FontWeight.W_700, color=TEXT),
                                 ft.Text("Send bytes exactly as entered", size=11, color=MUTED)],
                                spacing=1,
                            ),
                        ]
                    ),
                    ft.Row(
                        [
                            self.raw_hex,
                            ft.FilledButton(
                                "SEND RAW",
                                icon=ft.Icons.SEND_ROUNDED,
                                bgcolor=PRIMARY,
                                color=ft.Colors.WHITE,
                                style=self._button_style(),
                                on_click=self._send_raw_hex,
                            ),
                        ],
                        spacing=10,
                    ),
                    ft.Text("Use space-separated hexadecimal byte pairs. No framing is added automatically.", size=11, color=MUTED),
                ],
                spacing=12,
            ),
        )
        log_card = self._card(
            ft.Column(
                controls=[
                    ft.Row(
                        [
                            ft.Column(
                                [
                                    ft.Text("Communication log", size=18, weight=ft.FontWeight.W_700, color=TEXT),
                                    self.csv_log_status,
                                ],
                                spacing=1,
                            ),
                            ft.Container(expand=True),
                            ft.OutlinedButton(
                                "Clear",
                                icon=ft.Icons.DELETE_SWEEP_ROUNDED,
                                style=self._button_style(12),
                                on_click=self._clear_log,
                            ),
                        ]
                    ),
                    ft.Container(
                        content=self.log_view,
                        bgcolor="#FCFBFF",
                        border=BORDER_ALL(1, BORDER),
                        border_radius=14,
                        padding=4,
                        expand=True,
                    ),
                ],
                spacing=10,
                expand=True,
            ),
            height=560,
        )
        return ft.Column(
            controls=[
                self._eyebrow("Service tools"),
                self._title("Diagnostics"),
                ft.Text("Inspect decoded frames, startup notifications and exact UART bytes.", size=13, color=MUTED),
                raw_card,
                log_card,
            ],
            spacing=12,
            scroll=ft.ScrollMode.AUTO,
            expand=True,
        )

    def _populate_command_list(self, category: str | None = None) -> None:
        category = category or str(self.command_category.value or "All")
        self.command_list.controls.clear()
        for command_id in range(0x19):
            title, command_category, _summary = COMMAND_META[command_id]
            if category != "All" and command_category != category:
                continue
            selected = command_id == self.current_command_id
            blocked = command_id in BLOCKED_CURRENT_FIRMWARE
            self.command_list.controls.append(
                ft.Container(
                    data=command_id,
                    bgcolor=PRIMARY_SOFT if selected else "#FBFAFE",
                    border=BORDER_ALL(1, PRIMARY if selected else BORDER),
                    border_radius=12,
                    padding=11,
                    ink=True,
                    on_click=lambda e, cid=command_id: self._select_command(cid),
                    content=ft.Row(
                        [
                            ft.Container(
                                width=42,
                                height=32,
                                border_radius=9,
                                bgcolor=PRIMARY if selected else "#EDE9FE",
                                alignment=ALIGN_CENTER,
                                content=ft.Text(
                                    f"{command_id:02X}",
                                    color=ft.Colors.WHITE if selected else PRIMARY_DARK,
                                    size=11,
                                    weight=ft.FontWeight.W_700,
                                    font_family=MONO,
                                ),
                            ),
                            ft.Column(
                                [
                                    ft.Text(title, size=12, weight=ft.FontWeight.W_600, color=TEXT, no_wrap=True),
                                    ft.Text(command_category, size=10, color=MUTED),
                                ],
                                spacing=0,
                                expand=True,
                            ),
                            ft.Icon(
                                ft.Icons.BLOCK_ROUNDED if blocked else ft.Icons.CHEVRON_RIGHT_ROUNDED,
                                color=RED if blocked else MUTED,
                                size=17,
                            ),
                        ],
                        spacing=9,
                    ),
                )
            )

    def _filter_command_list(self, event: Any) -> None:
        self._populate_command_list(str(event.control.value))
        self.page.update()

    @staticmethod
    def _english_unit(unit: str) -> str:
        return unit.replace("0,1 s", "× 0.1 s").replace("×0,01", "× 0.01")

    def _select_command(self, command_id: int, update: bool = True) -> None:
        self.current_command_id = command_id
        spec = proto.COMMAND_BY_ID[command_id]
        title, _category, summary = COMMAND_META[command_id]
        self.command_id_text.value = f"CMD 0x{command_id:02X}"
        self.command_title_text.value = title
        self.command_summary_text.value = summary
        warning = ""
        if command_id == 0x0E:
            warning = "Referencing uses a fixed movement toward a mechanical stop; no end switch is checked."
        elif command_id == 0x13:
            warning = "UV output must only be enabled under safe, shielded conditions."
        self.command_warning.visible = bool(warning)
        self.command_warning.content.controls[1].value = warning
        self.command_send_button.disabled = command_id in BLOCKED_CURRENT_FIRMWARE

        self.command_fields.clear()
        controls: list[Any] = []
        for field in spec.fields:
            label = FIELD_LABELS.get(field.key, field.label)
            if field.choices:
                control = ft.Dropdown(
                    label=label,
                    value=str(field.default),
                    options=[
                        ft.DropdownOption(key=str(value), text=translated_choice(choice_label))
                        for choice_label, value in field.choices
                    ],
                    col={"sm": 12, "md": 6},
                    **dropdown_event(lambda e: self._update_command_preview()),
                    **self._input_style(),
                )
            else:
                unit = self._english_unit(field.unit)
                control = ft.TextField(
                    label=label,
                    value=str(field.default),
                    suffix=ft.Text(unit, size=10, color=MUTED) if unit else None,
                    keyboard_type=ft.KeyboardType.NUMBER,
                    on_change=lambda e: self._update_command_preview(),
                    col={"sm": 12, "md": 6},
                    **self._input_style(),
                )
            self.command_fields[field.key] = control
            controls.append(control)
        if controls:
            self.command_form_host.content = ft.ResponsiveRow(controls, spacing=10, run_spacing=10)
        else:
            self.command_form_host.content = ft.Container(
                bgcolor="#FAF7FF",
                border_radius=12,
                padding=16,
                content=ft.Row(
                    [ft.Icon(ft.Icons.CHECK_CIRCLE_OUTLINE_ROUNDED, color=GREEN),
                     ft.Text("This command has no payload parameters.", color=MUTED)],
                ),
            )
        self._populate_command_list()
        self._update_command_preview(update=False)
        if update:
            self.page.update()

    def _command_values(self) -> dict[str, Any]:
        return {key: control.value for key, control in self.command_fields.items()}

    def _current_command_frame(self) -> tuple[bytes, bytes, bytes]:
        spec = proto.COMMAND_BY_ID[self.current_command_id]
        payload = proto.encode_payload(spec, self._command_values())
        raw, wire = proto.build_command(self.current_command_id, self.sequence, payload, ACTIVE_PROFILE)
        return payload, raw, wire

    def _update_command_preview(self, update: bool = True) -> None:
        try:
            payload, raw, wire = self._current_command_frame()
            transmitted_length = int.from_bytes(raw[2:4], "big")
            adjusted = "" if transmitted_length == len(payload) else f" → {transmitted_length} B on UART"
            self.command_preview.value = (
                f"CMD     0x{self.current_command_id:02X}\n"
                f"SEQ     {self.sequence}\n"
                f"PAYLOAD {len(payload)} B logical{adjusted}\n"
                f"LEN     {proto.hex_bytes(raw[2:4])} (big-endian)\n"
                f"PROFILE CURRENT FIRMWARE\n"
                f"\n"
                f"LOGICAL PAYLOAD\n{proto.hex_bytes(payload) or '—'}\n\n"
                f"RAW BODY\n{proto.hex_bytes(raw)}\n\n"
                f"UART FRAME · {len(wire)} B\n{proto.hex_bytes(wire)}"
            )
        except Exception as exc:
            self.command_preview.value = f"Frame cannot be built yet:\n{exc}"
        if update:
            self.page.update()

    def _send_selected_command(self, _event: Any) -> None:
        if self.current_command_id in BLOCKED_CURRENT_FIRMWARE:
            self._toast(BLOCKED_CURRENT_FIRMWARE[self.current_command_id], error=True)
            return
        try:
            payload, _raw, _wire = self._current_command_frame()
            values = self._command_values()
        except Exception as exc:
            self._toast(f"Invalid command parameters: {exc}", error=True)
            return
        command_id = self.current_command_id
        if command_id in UNIT_LIMITED_COMMANDS and self.serial_port is not None:
            if self.input_unit is None:
                self._toast("Send Input units (0x00) first so the allowed amplitude and offset range is known.", error=True)
                return
            limit_error = unit_limit_error(command_id, values, self.input_unit)
            if limit_error:
                self._toast(f"Invalid command parameters: {limit_error}", error=True)
                return
        if command_id == 0x17:
            rate_error = magnetometer_rate_error(values)
            if rate_error:
                self._toast(f"Invalid command parameters: {rate_error}", error=True)
                return
        if command_id == 0x0E:
            self._confirm(
                "Reference motor?",
                "The motor will move a fixed number of steps toward the mechanical stop. The firmware does not check an end switch.",
                lambda: self._send_command(command_id, payload, command_title(command_id)),
            )
            return
        if command_id == 0x0F and int(str(values.get("steps", "0")), 0) > 1000:
            self._confirm(
                "Send a long motor movement?",
                f"The requested movement is {values['steps']} steps.",
                lambda: self._send_command(command_id, payload, command_title(command_id)),
            )
            return
        if command_id == 0x13 and int(str(values.get("light", "0")), 0) == 2 and int(str(values.get("action", "0")), 0) == 1:
            self._confirm(
                "Enable UV output?",
                "Confirm that the device is shielded and the workspace is safe.",
                lambda: self._send_command(command_id, payload, command_title(command_id)),
            )
            return
        self._send_command(command_id, payload, command_title(command_id))
        self.page.update()

    def _confirm(self, title: str, message: str, action: Callable[[], Any]) -> None:
        dialog: Any = None

        def cancel(_event: Any) -> None:
            self._close_dialog(dialog)
            self.page.update()

        def proceed(_event: Any) -> None:
            self._close_dialog(dialog)
            action()
            self.page.update()

        dialog = ft.AlertDialog(
            modal=True,
            icon=ft.Icon(ft.Icons.WARNING_AMBER_ROUNDED, color=AMBER, size=32),
            title=ft.Text(title, weight=ft.FontWeight.W_700),
            content=ft.Text(message, color=MUTED),
            actions=[
                ft.OutlinedButton("Cancel", on_click=cancel),
                ft.FilledButton("Confirm", bgcolor=PRIMARY, color=ft.Colors.WHITE, on_click=proceed),
            ],
            actions_alignment=ft.MainAxisAlignment.END,
            shape=ft.RoundedRectangleBorder(radius=18),
        )
        self._show_dialog(dialog)
        self.page.update()

    def _toast(self, message: str, error: bool = False) -> None:
        snack = ft.SnackBar(
            content=ft.Text(message, color=ft.Colors.WHITE),
            bgcolor=RED if error else PRIMARY_DARK,
            show_close_icon=True,
            behavior=ft.SnackBarBehavior.FLOATING,
        )
        self._show_dialog(snack)
        self.page.update()

    # ------------------------------------------------------------------
    # Serial connection and framing
    # ------------------------------------------------------------------

    @staticmethod
    def _set_chip(container: "ft.Container", text: str, color: str, background: str, icon: Any) -> None:
        container.bgcolor = background
        row = container.content
        if hasattr(row.controls[0], "icon"):
            row.controls[0].icon = icon
        else:
            row.controls[0].name = icon
        row.controls[0].color = color
        row.controls[1].value = text
        row.controls[1].color = color

    @staticmethod
    def _set_button_text(button: Any, text: str) -> None:
        if hasattr(button, "text"):
            button.text = text
        else:
            button.content = text

    def _show_dialog(self, dialog: Any) -> None:
        if hasattr(self.page, "show_dialog"):
            self.page.show_dialog(dialog)
        else:
            self.page.open(dialog)

    def _close_dialog(self, dialog: Any) -> None:
        if hasattr(self.page, "pop_dialog"):
            self.page.pop_dialog()
        else:
            self.page.close(dialog)

    def _refresh_ports(self, update: bool = True) -> None:
        selected = str(self.port_dropdown.value or "")
        devices: list[str] = []
        if list_ports is not None:
            try:
                devices = [port.device for port in sorted(list_ports.comports(), key=lambda item: item.device)]
            except Exception as exc:
                self.log("ERR", f"Serial-port scan failed: {exc}")
        if selected and selected not in devices:
            devices.insert(0, selected)
        self.port_dropdown.options = [ft.DropdownOption(key=device, text=device) for device in devices]
        if selected:
            self.port_dropdown.value = selected
        elif devices:
            self.port_dropdown.value = devices[0]
        if update:
            self.page.update()

    def _toggle_connection(self, _event: Any) -> None:
        if self.serial_port is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        if serial is None:
            self._toast(f"pyserial is missing. Install the requirements with: {INSTALL_COMMAND}", error=True)
            return
        port_name = str(self.port_dropdown.value or "").strip()
        if not port_name:
            self._toast("Select a serial port first.", error=True)
            return

        opened_port: Any = None
        try:
            # Configure DTR/RTS before opening. Some USB-UART adapters reset the
            # STM32 when either signal changes after the port has been opened.
            opened_port = serial.Serial()
            opened_port.port = port_name
            opened_port.baudrate = proto.BAUDRATE
            opened_port.bytesize = 8
            opened_port.parity = "N"
            opened_port.stopbits = 1
            opened_port.timeout = 0.05
            opened_port.write_timeout = 1.0
            opened_port.rtscts = False
            opened_port.dsrdtr = False
            opened_port.xonxoff = False
            opened_port.dtr = False
            opened_port.rts = False
            opened_port.open()
            self.serial_port = opened_port
        except Exception as exc:
            self.serial_port = None
            if opened_port is not None:
                try:
                    opened_port.close()
                except Exception:
                    pass
            self.log("ERR", f"Connection failed: {exc}")
            self._toast(f"Connection failed: {exc}", error=True)
            return

        self.frame_parser = proto.FrameStreamParser()
        self.ascii_rx_buffer.clear()
        self.command_queue.clear()
        self.awaiting_response = None
        self.response_deadline = 0.0
        self.response_fault = False
        self.sent_commands.clear()
        self.input_unit = None
        self.startup_reports.clear()
        self.waiting_for_ready = True
        self.startup_deadline = time.monotonic() + 1.5
        self._start_csv_log(port_name)
        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True, name="nanorobot-uart-rx")
        self.reader_thread.start()

        self._set_button_text(self.connect_button, "Disconnect")
        self.connect_button.icon = ft.Icons.LINK_OFF_ROUNDED
        self.connect_button.bgcolor = SURFACE
        self.connect_button.color = PRIMARY
        self._set_chip(self.connection_chip, "Waiting for STM32", AMBER, AMBER_SOFT, ft.Icons.HOURGLASS_TOP_ROUNDED)
        self.dashboard_connection_value.value = f"{port_name} · starting"
        self.log("INFO", f"Connected to {port_name} @ {proto.BAUDRATE} baud; waiting briefly for STM32 self-test.")
        self.page.update()

    def _disconnect(self, update: bool = True) -> None:
        if self.random_running:
            self._halt_random("DISCONNECTED", write_log=True, update=False)
        if self.serial_port is not None:
            self._send_command(
                0x07,
                bytes((3, 0)),
                "ALL coil outputs = STOP before disconnect",
                notify=False,
                enforce_delay=False,
                bypass_gate=True,
            )
        self.reader_stop.set()
        port = self.serial_port
        self.serial_port = None
        self.waiting_for_ready = False
        self.command_queue.clear()
        self.awaiting_response = None
        self.response_deadline = 0.0
        self.response_fault = False
        self.sent_commands.clear()
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        self._set_button_text(self.connect_button, "Connect")
        self.connect_button.icon = ft.Icons.LINK_ROUNDED
        self.connect_button.bgcolor = PRIMARY
        self.connect_button.color = ft.Colors.WHITE
        self._set_chip(self.connection_chip, "Disconnected", RED, RED_SOFT, ft.Icons.LINK_OFF_ROUNDED)
        self.dashboard_connection_value.value = "Offline"
        self.log("INFO", "Serial port disconnected.")
        self._write_csv_event("SYS", "DISCONNECT", description="Serial port disconnected")
        self._close_csv_log()
        if update and not self.shutting_down:
            self.page.update()

    def _reader_loop(self) -> None:
        while not self.reader_stop.is_set():
            port = self.serial_port
            if port is None:
                return
            try:
                first = port.read(1)
                if not first:
                    continue
                # A blocking read frequently returns the first 0x7E by itself
                # and the USB-UART driver exposes the rest a fraction later.
                # Collect a short burst for readable logs; the stream parser
                # remains fragmentation-safe even when bytes still split.
                data = bytearray(first)
                burst_deadline = time.monotonic() + 0.004
                while len(data) < 4096 and time.monotonic() < burst_deadline:
                    waiting = int(port.in_waiting)
                    if waiting:
                        data.extend(port.read(min(waiting, 4096 - len(data))))
                        burst_deadline = time.monotonic() + 0.002
                    else:
                        time.sleep(0.0005)
                self.rx_queue.put(("data", bytes(data)))
            except Exception as exc:
                self.rx_queue.put(("error", exc))
                return

    async def _serial_poll_loop(self) -> None:
        while not self.shutting_down:
            changed = False
            try:
                while True:
                    kind, value = self.rx_queue.get_nowait()
                    changed = True
                    if kind == "error":
                        self.log("ERR", f"UART read failed: {value}")
                        self._disconnect(update=False)
                    else:
                        self._handle_rx(bytes(value))
            except queue.Empty:
                pass
            except Exception as exc:
                self.log("ERR", f"RX parser failed: {exc}")
                self.frame_parser = proto.FrameStreamParser()
                changed = True

            if self.waiting_for_ready and time.monotonic() >= self.startup_deadline:
                if self.startup_reports:
                    reason = "not all start-up reports arrived"
                else:
                    reason = "no start-up reports within 1.5 s; assuming the board is already running"
                self._device_ready(reason, update=False)
                changed = True
            if self.awaiting_response is not None and time.monotonic() >= self.response_deadline:
                self._handle_response_timeout()
                changed = True
            if changed:
                try:
                    self.page.update()
                except Exception:
                    if self.shutting_down:
                        return
            await asyncio.sleep(0.05)

    def _handle_rx(self, data: bytes) -> None:
        self._write_csv_event("RX", "CHUNK", data=data, description=f"{len(data)} serial byte(s)")
        frames = self.frame_parser.feed(data)
        outside = self.frame_parser.drain_outside()
        for body in frames:
            if not body:
                # Repeated 0x7E delimiters are idle/boundary bytes, not frames.
                continue
            try:
                self.log("RX", f"Complete frame body · {len(body)} B\n{proto.hex_bytes(body)}")
                if CHECK_RX_CRC and len(body) >= 8 and not proto.frame_crc_ok(body):
                    expected = proto.hex_bytes(proto.crc32_mpeg2(body[:-4]).to_bytes(4, "big"))
                    self.log(
                        "ERR",
                        f"CRC mismatch: received {proto.hex_bytes(body[-4:])}, expected {expected}. "
                        f"The frame may be damaged.",
                    )
                    self._write_csv_event("RX", "CRC_ERROR", data=body, description=f"expected CRC {expected}")
                normal = normal_response_fields(body)
                tracked: QueuedCommand | None = None
                if normal is not None:
                    _response_id, sequence, _status, _payload = normal
                    tracked = self.sent_commands.get(sequence) or self.awaiting_response
                decoded = decode_response_en(body, tracked.command_id if tracked is not None else None)
                self.log("RX", decoded)
                self.dashboard_last_rx.value = response_name(
                    body[0], tracked.command_id if tracked is not None else None
                )
                self._write_csv_event(
                    "RX",
                    "FRAME",
                    data=body,
                    command_id=tracked.command_id if tracked is not None else None,
                    response_id=normal[0] if normal is not None else body[0],
                    sequence=normal[1] if normal is not None else None,
                    status=normal[2] if normal is not None else (body[1] if len(body) > 1 else None),
                    description=decoded,
                )
                notification = notification_fields(body)
                if normal is not None:
                    self._handle_command_response(*normal)
                elif (
                    notification is not None
                    and notification[0] == 0xFF
                    and notification[1] != SUCCESS_STATUS
                    and notification[1] not in STARTUP_STATUSES
                ):
                    self._handle_error_notification(notification[1])
                elif notification is not None and notification[0] == 0xFF:
                    self._handle_startup_report(notification[1])
            except Exception as exc:
                self.log("ERR", f"Cannot decode frame: {exc} · RAW {proto.hex_bytes(body)}")
                self._write_csv_event("RX", "INVALID_FRAME", data=body, description=str(exc))
        # Process raw text after frames so a READY marker in the same USB read
        # correctly closes a self-test sequence opened by a notification.
        if outside:
            self._handle_ascii_rx(outside)

    def _handle_ascii_rx(self, data: bytes) -> None:
        self.ascii_rx_buffer.extend(data)
        if len(self.ascii_rx_buffer) > 512:
            del self.ascii_rx_buffer[:-256]
        printable = "".join(
            chr(byte) if byte in b"\r\n\t" or 32 <= byte < 127 else f"\\x{byte:02X}"
            for byte in data
        ).strip()
        if printable:
            self.log("RX", f"Text: {printable}")
            self.dashboard_last_rx.value = printable[:34]
        if b"STM32 ready!!!" in self.ascii_rx_buffer:
            self._device_ready("STM32 ready!!!", update=False)
            self.ascii_rx_buffer.clear()

    def _device_ready(self, reason: str, update: bool = True) -> None:
        was_waiting = self.waiting_for_ready
        self.waiting_for_ready = False
        if self.serial_port is not None:
            port_name = str(self.port_dropdown.value or "UART")
            self._set_chip(self.connection_chip, "STM32 ready", GREEN, GREEN_SOFT, ft.Icons.CHECK_CIRCLE_ROUNDED)
            self.dashboard_connection_value.value = f"{port_name} · ready"
        if was_waiting:
            self.log("INFO", f"Device startup completed ({reason}).")
        if self.startup_reports:
            self._log_startup_summary()
        self._dispatch_next_command()
        if update:
            self.page.update()

    def _handle_startup_report(self, status: int) -> None:
        if status not in STARTUP_STATUSES:
            return
        if not self.startup_reports:
            self._handle_board_start()
        name, sensor_ok = startup_sensor(status)
        self.startup_reports[name] = sensor_ok
        self.startup_deadline = time.monotonic() + 2.0
        if len(self.startup_reports) >= STARTUP_SENSOR_COUNT:
            self._device_ready(f"all {STARTUP_SENSOR_COUNT} start-up reports received", update=False)

    def _handle_board_start(self) -> None:
        self.waiting_for_ready = True
        self.startup_deadline = time.monotonic() + 2.0
        self.log("INFO", "Board start-up detected: coil outputs are off and the motors need a new reference.")
        lost = self.awaiting_response
        if lost is not None:
            self.sent_commands.pop(lost.sequence, None)
            self._apply_command_result(lost, None)
            self._report_command_failure(lost, None, "the board restarted before replying")
            self.awaiting_response = None
            self.response_deadline = 0.0
        self.sent_commands.clear()
        self.motor_active = False
        self.active_coil_outputs.clear()
        self.coil_state_unknown = False
        self.coil_state_value.value = "Outputs stopped (board restarted)"
        if self.random_running:
            self._halt_random("ERROR", write_log=True, update=False)
        self.input_unit = FIRMWARE_DEFAULT_UNIT

    def _log_startup_summary(self) -> None:
        failed = [name for name, sensor_ok in sorted(self.startup_reports.items()) if not sensor_ok]
        summary = f"Board self-test: {len(self.startup_reports)} of {STARTUP_SENSOR_COUNT} sensors reported"
        if failed:
            self.log("ERR", f"{summary} · FAILED: {', '.join(failed)}.")
        else:
            self.log("INFO", f"{summary} · all OK.")
        self.startup_reports.clear()

    def _handle_command_response(
        self,
        response_id: int,
        sequence: int,
        status: int,
        _payload: bytes,
    ) -> None:
        gate_command = self.awaiting_response
        exact_command = self.sent_commands.get(sequence)
        if exact_command is not None and exact_command.expected_response_id != response_id:
            exact_command = None

        if exact_command is None:
            if gate_command is None:
                self.log("INFO", f"Unsolicited response 0x{response_id:02X}, SEQ {sequence}; TX gate unchanged.")
                return
            self.sent_commands.pop(gate_command.sequence, None)
            self._apply_command_result(gate_command, None)
            self._report_command_failure(
                gate_command,
                None,
                f"unexpected reply ID 0x{response_id:02X}, SEQ {sequence} "
                f"(the firmware usually answers like this when it rejects the parameters)",
            )
            self._release_tx_gate()
            return

        self.sent_commands.pop(exact_command.sequence, None)
        self._apply_command_result(exact_command, status)
        if status != SUCCESS_STATUS:
            self._report_command_failure(exact_command, status)
        if exact_command is gate_command:
            self._release_tx_gate()

    def _handle_error_notification(self, status: int) -> None:
        command = self.awaiting_response
        if command is None:
            self.log("ERR", f"Firmware error notification: {STATUS_EN.get(status, f'status 0x{status:02X}')}.")
            return
        self.sent_commands.pop(command.sequence, None)
        self._apply_command_result(command, status)
        self._report_command_failure(command, status)
        self._release_tx_gate()

    def _report_command_failure(self, command: QueuedCommand, status: int | None, reason: str | None = None) -> None:
        detail = reason or STATUS_EN.get(status, f"status 0x{status:02X}")
        message = f"CMD 0x{command.command_id:02X} ({command_title(command.command_id)}) failed: {detail}."
        self.log("ERR", message)
        if command.command_id in COIL_COMMAND_IDS and status in COIL_STOP_STATUSES:
            self.active_coil_outputs.clear()
            self.coil_state_unknown = False
            self.coil_state_value.value = "Outputs stopped by firmware error"
            self.log("ERR", "The firmware switched all coil outputs off after this error.")
        self._discard_queued_commands("after the failure")
        if command.source == "random" and self.random_running:
            self._halt_random("ERROR", write_log=True, update=False)
        if command.notify:
            self._toast(message, error=True)

    def _discard_queued_commands(self, reason: str) -> None:
        discarded = self.command_queue[:]
        self.command_queue.clear()
        if not discarded:
            return
        if any(queued.command_id == 0x00 for queued in discarded):
            self.input_unit = None
        self.log("ERR", f"{len(discarded)} queued command(s) discarded {reason}.")

    def _release_tx_gate(self) -> None:
        self.awaiting_response = None
        self.response_deadline = 0.0
        if not self.response_fault and self.serial_port is not None:
            self._set_chip(
                self.connection_chip,
                "STM32 ready",
                GREEN,
                GREEN_SOFT,
                ft.Icons.CHECK_CIRCLE_ROUNDED,
            )
        self._dispatch_next_command()

    def _apply_command_result(self, command: QueuedCommand, status: int | None) -> None:
        if command.command_id in MOTOR_COMMAND_IDS:
            self.motor_active = False
            self.log("INFO", f"Motor operation finished · SEQ {command.sequence}.")

        if command.command_id == 0x00:
            if status == SUCCESS_STATUS:
                self.active_coil_outputs.clear()
                self.coil_state_value.value = "Outputs stopped"
                self.coil_state_unknown = False
            else:
                self.input_unit = None
            return

        if command.command_id != 0x07 or len(command.payload) < 2 or status != SUCCESS_STATUS:
            return
        output, state = command.payload[0], command.payload[1]
        if output == 3:
            if state == 1:
                self.active_coil_outputs = {0, 1, 2, 4, 5}
            else:
                self.active_coil_outputs.clear()
                self.coil_state_unknown = False
        elif state == 1:
            self.active_coil_outputs.add(output)
        else:
            self.active_coil_outputs.discard(output)
        if self.active_coil_outputs:
            self.coil_state_value.value = "Outputs running · motor locked"
        elif self.coil_state_unknown:
            self.coil_state_value.value = "Output state unknown · motor locked"
        else:
            self.coil_state_value.value = "Outputs stopped"

    def _handle_response_timeout(self) -> None:
        command = self.awaiting_response
        if command is None:
            return
        self.awaiting_response = None
        self.response_deadline = 0.0
        self.response_fault = True
        self.input_unit = None
        queued = len(self.command_queue)
        self.command_queue.clear()
        if command.command_id == 0x07 and len(command.payload) >= 2 and command.payload[1] == 1:
            self.coil_state_unknown = True
            self.coil_state_value.value = "Output state unknown · motor locked"
        self._set_chip(
            self.connection_chip,
            f"No response · SEQ {command.sequence}",
            RED,
            RED_SOFT,
            ft.Icons.ERROR_OUTLINE_ROUNDED,
        )
        self.log(
            "ERR",
            f"No response for CMD 0x{command.command_id:02X}, SEQ {command.sequence}. "
            f"TX blocked; {queued} queued command(s) discarded. Reset SEQ to recover.",
        )
        self._write_csv_event(
            "SYS",
            "RESPONSE_TIMEOUT",
            command_id=command.command_id,
            response_id=command.expected_response_id,
            sequence=command.sequence,
            description=f"No response; {queued} queued command(s) discarded",
        )
        if self.random_running:
            self._halt_random("UART TIMEOUT", write_log=True, update=False)

    def _start_csv_log(self, port_name: str) -> None:
        self._close_csv_log()
        try:
            log_directory = Path(__file__).resolve().parent / "uart_logs"
            log_directory.mkdir(parents=True, exist_ok=True)
            safe_port = "".join(char if char.isalnum() or char in "-_" else "_" for char in port_name)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            self.csv_path = log_directory / f"uart_{safe_port}_{stamp}.csv"
            self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8-sig")
            fieldnames = [
                "timestamp",
                "direction",
                "event",
                "command_id",
                "response_id",
                "sequence",
                "status",
                "length",
                "hex",
                "description",
            ]
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()
            self.csv_file.flush()
            self.csv_log_status.value = f"CSV: {self.csv_path}"
            self._write_csv_event("SYS", "CONNECT", description=f"Connected to {port_name}")
        except Exception as exc:
            self.csv_file = None
            self.csv_writer = None
            self.csv_path = None
            self.csv_log_status.value = f"CSV logging unavailable: {exc}"
            self.log("ERR", f"Cannot create CSV UART log: {exc}")

    def _write_csv_event(
        self,
        direction: str,
        event: str,
        *,
        data: bytes = b"",
        command_id: int | None = None,
        response_id: int | None = None,
        sequence: int | None = None,
        status: int | None = None,
        description: str = "",
    ) -> None:
        writer = self.csv_writer
        output = self.csv_file
        if writer is None or output is None:
            return
        try:
            with self.csv_lock:
                writer.writerow(
                    {
                        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                        "direction": direction,
                        "event": event,
                        "command_id": "" if command_id is None else f"0x{command_id:02X}",
                        "response_id": "" if response_id is None else f"0x{response_id:02X}",
                        "sequence": "" if sequence is None else sequence,
                        "status": "" if status is None else f"0x{status:02X}",
                        "length": len(data),
                        "hex": " ".join(f"{byte:02X}" for byte in data),
                        "description": description.replace("\n", " | "),
                    }
                )
                output.flush()
        except Exception as exc:
            self.csv_writer = None
            self.csv_log_status.value = f"CSV logging stopped: {exc}"
            self.log("ERR", f"CSV UART logging failed: {exc}")

    def _close_csv_log(self) -> None:
        output = self.csv_file
        self.csv_writer = None
        self.csv_file = None
        if output is not None:
            try:
                output.flush()
                output.close()
            except Exception:
                pass

    def _tx_delay_seconds(self) -> float:
        try:
            delay_ms = float(str(self.tx_delay_field.value or "0").strip().replace(",", "."))
        except ValueError as exc:
            raise ValueError("TX delay must be a number between 0 and 5000 ms.") from exc
        if not math.isfinite(delay_ms) or not 0 <= delay_ms <= 5000:
            raise ValueError("TX delay must be between 0 and 5000 ms.")
        return delay_ms / 1000.0

    def _reset_sequence(self, _event: Any = None) -> None:
        if self.awaiting_response is not None:
            self._toast(
                f"Cannot reset SEQ while waiting for response {self.awaiting_response.sequence}.",
                error=True,
            )
            return
        if self.motor_active:
            self._toast("Cannot reset SEQ while a motor operation may still be running.", error=True)
            return
        with self.sequence_lock:
            self.sequence = 0
            self.sequence_text.value = "SEQ 0"
            self._update_command_preview(update=False)
        discarded = len(self.command_queue)
        self.command_queue.clear()
        self.sent_commands.clear()
        self.response_fault = False
        if self.serial_port is not None and not self.waiting_for_ready:
            self._set_chip(
                self.connection_chip,
                "STM32 ready",
                GREEN,
                GREEN_SOFT,
                ft.Icons.CHECK_CIRCLE_ROUNDED,
            )
        self.log("INFO", f"UART sequence number reset to 0; {discarded} queued command(s) discarded.")
        self.page.update()

    @staticmethod
    def _is_coil_enable(command_id: int, payload: bytes) -> bool:
        return command_id == 0x07 and len(payload) >= 2 and payload[1] == 1

    def _motor_operation_pending(self) -> bool:
        return (
            self.motor_active
            or (self.awaiting_response is not None and self.awaiting_response.command_id in MOTOR_COMMAND_IDS)
            or any(command.command_id in MOTOR_COMMAND_IDS for command in self.command_queue)
        )

    def _coil_enable_pending(self) -> bool:
        return (
            (self.awaiting_response is not None and self._is_coil_enable(
                self.awaiting_response.command_id, self.awaiting_response.payload
            ))
            or any(self._is_coil_enable(command.command_id, command.payload) for command in self.command_queue)
        )

    def _validate_actuator_interlock(
        self,
        command_id: int,
        payload: bytes,
        *,
        notify: bool,
        bypass_gate: bool,
    ) -> bool:
        message = ""
        if command_id in MOTOR_COMMAND_IDS:
            if self.motor_active:
                message = "Motor command blocked: another motor operation is still running."
            elif self.active_coil_outputs or self.coil_state_unknown or self._coil_enable_pending():
                message = "Motor command blocked: stop all coil outputs and wait for their response first."
        elif self._is_coil_enable(command_id, payload) and self._motor_operation_pending():
            message = "Coil enable blocked: wait until the motor operation has finished."
        if not message or (bypass_gate and command_id == 0x07 and len(payload) >= 2 and payload[1] == 0):
            return True
        self.log("ERR", message)
        if notify:
            self._toast(message, error=True)
        return False

    def _write_wire_now(
        self,
        wire: bytes,
        description: str,
        notify: bool = True,
        enforce_delay: bool = True,
        command: QueuedCommand | None = None,
    ) -> bool:
        try:
            delay = max(self._tx_delay_seconds(), MIN_FRAME_GAP_S) if enforce_delay else MIN_FRAME_GAP_S
        except ValueError as exc:
            self.log("ERR", str(exc))
            if notify:
                self._toast(str(exc), error=True)
            return False
        try:
            with self.tx_lock:
                port = self.serial_port
                if port is None:
                    return False
                remaining = self.last_tx_time + delay - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                written = port.write(wire)
                port.flush()
                self.last_tx_time = time.monotonic()
            self.log("TX", f"{description} · {written} B\n{proto.hex_bytes(wire)}")
            self._write_csv_event(
                "TX",
                "FRAME" if command is not None else "RAW",
                data=wire,
                command_id=command.command_id if command is not None else None,
                response_id=command.expected_response_id if command is not None else None,
                sequence=command.sequence if command is not None else None,
                description=description,
            )
            return written == len(wire)
        except Exception as exc:
            self.log("ERR", f"UART write failed: {exc}")
            if notify:
                self._toast(f"UART write failed: {exc}", error=True)
            return False

    def _send_command(
        self,
        command_id: int,
        payload: bytes,
        description: str,
        notify: bool = True,
        enforce_delay: bool = True,
        source: str = "manual",
        bypass_gate: bool = False,
    ) -> bool:
        if self.serial_port is None:
            if notify:
                self._toast("UART is not connected.", error=True)
            return False
        if self.response_fault and not bypass_gate:
            if notify:
                self._toast("TX is blocked because the previous command had no response. Reset SEQ to recover.", error=True)
            return False
        if len(self.command_queue) >= MAX_COMMAND_QUEUE and not bypass_gate:
            if notify:
                self._toast("UART command queue is full.", error=True)
            return False
        if not self._validate_actuator_interlock(
            command_id,
            payload,
            notify=notify,
            bypass_gate=bypass_gate,
        ):
            return False
        with self.sequence_lock:
            sequence = self.sequence
            try:
                _raw, wire = proto.build_command(command_id, sequence, payload, ACTIVE_PROFILE)
            except Exception as exc:
                self.log("ERR", f"Cannot build CMD 0x{command_id:02X}: {exc}")
                if notify:
                    self._toast(f"Cannot build command: {exc}", error=True)
                return False
            command = QueuedCommand(
                command_id=command_id,
                sequence=sequence,
                payload=payload,
                wire=wire,
                description=f"CMD 0x{command_id:02X} · {description} · SEQ {sequence}",
                notify=notify,
                enforce_delay=enforce_delay,
                source=source,
                bypass_gate=bypass_gate,
            )
            self.sequence = (sequence + 1) & 0xFF
            self.sequence_text.value = f"SEQ {self.sequence}"
            self._update_command_preview(update=False)

        if bypass_gate:
            if not self._write_wire_now(
                command.wire,
                command.description,
                notify=notify,
                enforce_delay=enforce_delay,
                command=command,
            ):
                with self.sequence_lock:
                    if self.sequence == ((sequence + 1) & 0xFF):
                        self.sequence = sequence
                        self.sequence_text.value = f"SEQ {self.sequence}"
                        self._update_command_preview(update=False)
                return False
            self.sent_commands[sequence] = command
            if self.awaiting_response is None and not self.response_fault:
                self.awaiting_response = command
                self.response_deadline = time.monotonic() + command.response_timeout
                self._set_chip(
                    self.connection_chip,
                    f"Waiting · SEQ {command.sequence}",
                    AMBER,
                    AMBER_SOFT,
                    ft.Icons.HOURGLASS_TOP_ROUNDED,
                )
            return True

        self.command_queue.append(command)
        if command_id == 0x00 and payload and payload[0] in UNIT_LIMITS:
            self.input_unit = payload[0]
        if self.awaiting_response is not None or self.waiting_for_ready:
            self.log("INFO", f"Queued until the previous response arrives: {command.description}")
        self._dispatch_next_command()
        return True

    def _dispatch_next_command(self) -> None:
        if (
            self.serial_port is None
            or self.waiting_for_ready
            or self.response_fault
            or self.awaiting_response is not None
            or not self.command_queue
        ):
            return
        command = self.command_queue.pop(0)
        if command.command_id in MOTOR_COMMAND_IDS:
            self.motor_active = True
        if not self._write_wire_now(
            command.wire,
            command.description,
            notify=command.notify,
            enforce_delay=command.enforce_delay,
            command=command,
        ):
            if command.command_id in MOTOR_COMMAND_IDS:
                self.motor_active = False
            self.command_queue.clear()
            return
        self.awaiting_response = command
        self.response_deadline = time.monotonic() + command.response_timeout
        self.sent_commands[command.sequence] = command
        if command.command_id in MOTOR_COMMAND_IDS:
            self.log(
                "INFO",
                f"Motor operation running · waiting up to {command.response_timeout:.0f} s. "
                f"The firmware handles no other command until it finishes.",
            )
        self._set_chip(
            self.connection_chip,
            f"Waiting · SEQ {command.sequence}",
            AMBER,
            AMBER_SOFT,
            ft.Icons.HOURGLASS_TOP_ROUNDED,
        )

    def _send_ping(self, _event: Any = None) -> None:
        if self._send_command(0x0D, b"", "connection check"):
            self.page.update()

    def _send_raw_hex(self, _event: Any = None) -> None:
        try:
            cleaned = str(self.raw_hex.value or "").replace("0x", "").replace(",", " ").replace(";", " ")
            wire = bytes.fromhex(cleaned)
            if not wire:
                raise ValueError("the field is empty")
        except ValueError as exc:
            self._toast(f"Invalid hexadecimal bytes: {exc}", error=True)
            return
        parser = proto.FrameStreamParser()
        frames = parser.feed(wire)
        outside = parser.drain_outside()
        if len(frames) != 1 or outside or parser.in_frame:
            self._toast("Raw TX requires exactly one complete framed command while response gating is enabled.", error=True)
            return
        body = frames[0]
        if len(body) < 8:
            self._toast("Raw command frame is too short.", error=True)
            return
        command_id, sequence = body[0], body[1]
        payload_length = int.from_bytes(body[2:4], "big")
        if command_id not in EXPECTED_RESPONSE_IDS or len(body) != 4 + payload_length + 4:
            self._toast("Raw TX must contain one structurally valid known command.", error=True)
            return
        payload = body[4:4 + payload_length]
        if self.response_fault or self.awaiting_response is not None or self.waiting_for_ready or self.command_queue:
            self._toast("Wait for the current response or reset SEQ after a timeout.", error=True)
            return
        if not self._validate_actuator_interlock(
            command_id,
            payload,
            notify=True,
            bypass_gate=False,
        ):
            return
        command = QueuedCommand(
            command_id=command_id,
            sequence=sequence,
            payload=payload,
            wire=wire,
            description=f"RAW CMD 0x{command_id:02X} · SEQ {sequence}",
            source="raw",
        )
        self.command_queue.append(command)
        self._dispatch_next_command()
        self.page.update()

    # ------------------------------------------------------------------
    # Manual field, coil and motor controls
    # ------------------------------------------------------------------

    @staticmethod
    def _integer_control(control: Any, label: str, minimum: int, maximum: int) -> int:
        try:
            parsed = float(str(control.value or "").strip().replace(",", "."))
        except ValueError as exc:
            raise ValueError(f"{label}: enter a valid number.") from exc
        if not parsed.is_integer():
            raise ValueError(f"{label}: enter a whole number.")
        value = int(parsed)
        if not minimum <= value <= maximum:
            raise ValueError(f"{label}: allowed range is {minimum}…{maximum}.")
        return value

    def _manual_field_values(self) -> tuple[int, dict[str, int]]:
        unit = int(str(self.manual_unit.value or "2"), 0)
        if unit not in UNIT_LIMITS:
            raise ValueError("Input units: select mV, mA or µT.")
        unit_name, limit = UNIT_LIMITS[unit]
        values = {
            "azimuth": self._integer_control(self.manual_azimuth, "Azimuth", 0, 360),
            "elevation": self._integer_control(self.manual_elevation, "Elevation", -180, 180),
            "amplitude": self._integer_control(self.manual_amplitude, f"Amplitude ({unit_name})", 0, limit),
            "frequency": self._integer_control(self.manual_frequency, "Rotation frequency", 0, 100),
            "offset_x": self._integer_control(self.manual_offset_x, f"Offset X ({unit_name})", -limit, limit),
            "offset_y": self._integer_control(self.manual_offset_y, f"Offset Y ({unit_name})", -limit, limit),
            "offset_z": self._integer_control(self.manual_offset_z, f"Offset Z ({unit_name})", -limit, limit),
            "ramp": self._integer_control(self.manual_ramp, "Ramp", 0, 200),
        }
        return unit, values

    def _update_manual_field_summary(self, _event: Any = None) -> None:
        unit_labels = {0: "mV", 1: "mA", 2: "µT"}
        try:
            unit, values = self._manual_field_values()
            self.manual_field_summary.value = (
                f"{values['azimuth']}° azimuth · {values['elevation']:+d}° elevation · "
                f"{values['amplitude']} {unit_labels[unit]} · {values['frequency']} Hz"
            )
            self.manual_field_status.value = (
                f"Ramp {values['ramp']} · offsets "
                f"X {values['offset_x']:+d}, Y {values['offset_y']:+d}, Z {values['offset_z']:+d}"
            )
            self.manual_field_status.color = MUTED
        except Exception as exc:
            self.manual_field_summary.value = "Complete the field parameters"
            self.manual_field_status.value = str(exc)
            self.manual_field_status.color = RED
        self.page.update()

    def _apply_manual_field(self, _event: Any = None) -> None:
        try:
            unit, values = self._manual_field_values()
            payload = proto.encode_payload(proto.COMMAND_BY_ID[0x06], values)
        except Exception as exc:
            self._toast(f"Invalid field parameters: {exc}", error=True)
            return
        unit_name = {0: "mV", 1: "mA", 2: "µT"}[unit]
        if self.input_unit != unit and not self._send_command(0x00, bytes((unit,)), f"input units = {unit_name}"):
            return
        if not self._send_command(
            0x06,
            payload,
            f"field azimuth {values['azimuth']}°, elevation {values['elevation']}°",
        ):
            return
        if bool(self.manual_enable_outputs.value):
            if self._send_command(0x07, bytes((3, 1)), "ALL coil outputs = RUN"):
                self.coil_state_value.value = "Enable queued · awaiting response"
        self.manual_field_status.value = f"Field command chain queued · next SEQ {self.sequence}"
        self.manual_field_status.color = AMBER
        self.page.update()

    def _zero_manual_field(self, _event: Any = None) -> None:
        self.manual_amplitude.value = "0"
        self.manual_offset_x.value = "0"
        self.manual_offset_y.value = "0"
        self.manual_offset_z.value = "0"
        self._update_manual_field_summary()
        self._apply_manual_field()

    def _set_manual_motor_steps(self, steps: int) -> None:
        self.manual_motor_steps.value = str(steps)
        self.page.update()

    def _manual_motor_selection(self) -> tuple[int, int, int]:
        motor = int(str(self.manual_motor_select.value or "0"), 0)
        selected = list(self.manual_motor_direction.selected or ["0"])
        direction = int(str(selected[0] if selected else "0"), 0)
        steps = self._integer_control(self.manual_motor_steps, "Steps", 0, 5000)
        if motor not in (0, 1):
            raise ValueError("Motor must be 0 or 1.")
        if direction not in (0, 1):
            raise ValueError("Direction must be 0 or 1.")
        return motor, direction, steps

    def _move_manual_motor(self, _event: Any = None) -> None:
        try:
            motor, direction, steps = self._manual_motor_selection()
        except Exception as exc:
            self._toast(f"Invalid motor movement: {exc}", error=True)
            return

        def send() -> None:
            payload = struct.pack(">BBH", motor, direction, steps)
            self._send_command(0x0F, payload, f"motor {motor}, direction {direction}, {steps} steps")

        if steps > 1000:
            self._confirm("Send a long motor movement?", f"The requested movement is {steps} steps.", send)
        else:
            send()
            self.page.update()

    def _confirm_manual_motor_reference(self, _event: Any = None) -> None:
        motor = int(str(self.manual_motor_select.value or "0"), 0)
        self._confirm(
            "Reference motor?",
            "The firmware drives a fixed number of steps toward the mechanical stop and does not check an end switch.",
            lambda: self._reference_motor(motor),
        )

    def _manual_selected_sample(self) -> int:
        sample = int(str(self.manual_motor_sample.value or "0"), 0)
        if not 0 <= sample <= 19:
            raise ValueError("Sample position must be between 0 and 19.")
        return sample

    def _manual_go_to_sample(self, _event: Any = None) -> None:
        try:
            sample = self._manual_selected_sample()
        except ValueError as exc:
            self._toast(str(exc), error=True)
            return
        if self._send_command(0x10, bytes((sample,)), f"move to sample {sample:02d}"):
            self.page.update()

    def _confirm_manual_save_sample(self, _event: Any = None) -> None:
        try:
            sample = self._manual_selected_sample()
        except ValueError as exc:
            self._toast(str(exc), error=True)
            return
        self._confirm(
            "Save the current motor position?",
            f"The current exchange-motor position will be stored as sample {sample:02d} in firmware RAM.",
            lambda: self._save_sample(sample),
        )

    # ------------------------------------------------------------------
    # Shared coil and Random Motion motor actions
    # ------------------------------------------------------------------

    def _emergency_stop(self, _event: Any = None) -> None:
        self._halt_random("EMERGENCY STOP", write_log=True, update=False)
        self._discard_queued_commands("by EMERGENCY STOP")
        if self._send_command(
            0x07,
            bytes((3, 0)),
            "ALL coil outputs = STOP",
            enforce_delay=False,
            bypass_gate=True,
        ):
            self.coil_state_value.value = "Emergency STOP sent · awaiting response"
        self.page.update()

    def _confirm_coils_on(self, _event: Any) -> None:
        self._confirm(
            "Enable all coil outputs?",
            "Confirm that field parameters and power electronics are ready.",
            lambda: self._set_all_coils(True),
        )

    def _set_all_coils(self, enabled: bool) -> None:
        state = 1 if enabled else 0
        label = "RUN" if enabled else "STOP"
        if self._send_command(0x07, bytes((3, state)), f"ALL coil outputs = {label}"):
            self.coil_state_value.value = f"{label} queued · awaiting response"
        self.page.update()

    def _set_motor_steps(self, steps: int) -> None:
        self.motor_steps.value = str(steps)
        self.page.update()

    def _motor_selection(self) -> tuple[int, int, int]:
        motor = int(str(self.motor_select.value or "0"), 0)
        selected = list(self.motor_direction.selected or ["0"])
        direction = int(str(selected[0] if selected else "0"), 0)
        steps = int(str(self.motor_steps.value or "0").strip(), 0)
        if motor not in (0, 1):
            raise ValueError("Motor must be 0 or 1.")
        if direction not in (0, 1):
            raise ValueError("Direction must be 0 or 1.")
        if not 0 <= steps <= 5000:
            raise ValueError("Steps must be between 0 and 5000.")
        return motor, direction, steps

    def _move_motor(self, _event: Any) -> None:
        try:
            motor, direction, steps = self._motor_selection()
        except Exception as exc:
            self._toast(f"Invalid motor movement: {exc}", error=True)
            return

        def send() -> None:
            payload = struct.pack(">BBH", motor, direction, steps)
            self._send_command(0x0F, payload, f"motor {motor}, direction {direction}, {steps} steps")

        if steps > 1000:
            self._confirm("Send a long motor movement?", f"The requested movement is {steps} steps.", send)
        else:
            send()
            self.page.update()

    def _confirm_motor_reference(self, _event: Any) -> None:
        motor = int(str(self.motor_select.value or "0"), 0)
        self._confirm(
            "Reference motor?",
            "The firmware drives a fixed number of steps toward the mechanical stop and does not check an end switch.",
            lambda: self._reference_motor(motor),
        )

    def _reference_motor(self, motor: int) -> None:
        self._send_command(0x0E, bytes((motor,)), f"reference motor {motor}")

    def _selected_sample(self) -> int:
        sample = int(str(self.motor_sample.value or "0"), 0)
        if not 0 <= sample <= 19:
            raise ValueError("Sample position must be between 0 and 19.")
        return sample

    def _go_to_sample(self, _event: Any = None) -> None:
        try:
            sample = self._selected_sample()
        except ValueError as exc:
            self._toast(str(exc), error=True)
            return
        if self._send_command(0x10, bytes((sample,)), f"move to sample {sample:02d}"):
            self.page.update()

    def _confirm_save_sample(self, _event: Any = None) -> None:
        try:
            sample = self._selected_sample()
        except ValueError as exc:
            self._toast(str(exc), error=True)
            return
        self._confirm(
            "Save the current motor position?",
            f"The current exchange-motor position will be stored as sample {sample:02d} in firmware RAM.",
            lambda: self._save_sample(sample),
        )

    def _save_sample(self, sample: int) -> None:
        self._send_command(0x11, bytes((sample,)), f"save sample {sample:02d}")

    # ------------------------------------------------------------------
    # Random Motion
    # ------------------------------------------------------------------

    def _random_mode_changed(self, _event: Any = None) -> None:
        disabled = str(self.random_mode.value or "smooth") == "random"
        self.random_step_az.disabled = disabled
        self.random_step_el.disabled = disabled
        self.page.update()

    @staticmethod
    def _float_control(control: Any, label: str) -> float:
        try:
            return float(str(control.value or "").strip().replace(",", "."))
        except ValueError as exc:
            raise ValueError(f"{label}: enter a valid number.") from exc

    def _random_config_from_form(self) -> dict[str, Any]:
        config: dict[str, Any] = {
            "mode": str(self.random_mode.value or "smooth"),
            "duration": self._float_control(self.random_duration, "Duration"),
            "interval": self._float_control(self.random_interval, "Change interval"),
            "az_min": self._float_control(self.random_az_min, "Azimuth minimum"),
            "az_max": self._float_control(self.random_az_max, "Azimuth maximum"),
            "el_min": self._float_control(self.random_el_min, "Elevation minimum"),
            "el_max": self._float_control(self.random_el_max, "Elevation maximum"),
            "step_az": self._float_control(self.random_step_az, "Azimuth step"),
            "step_el": self._float_control(self.random_step_el, "Elevation step"),
            "amplitude": self._float_control(self.random_amplitude, "Amplitude"),
            "frequency": self._float_control(self.random_frequency, "Rotation frequency"),
            "ramp": self._float_control(self.random_ramp, "Ramp"),
            "enable_outputs": bool(self.random_enable_outputs.value),
            "power_off": bool(self.random_power_off.value),
        }
        if config["duration"] <= 0:
            raise ValueError("Duration must be greater than zero.")
        if config["interval"] < 0.1:
            raise ValueError("Change interval must be at least 0.1 s.")
        if not 0 <= config["az_min"] <= config["az_max"] <= 360:
            raise ValueError("Azimuth range must satisfy 0 ≤ MIN ≤ MAX ≤ 360°.")
        if not -180 <= config["el_min"] <= config["el_max"] <= 180:
            raise ValueError("Elevation range must satisfy −180 ≤ MIN ≤ MAX ≤ 180°.")
        if config["step_az"] < 0 or config["step_el"] < 0:
            raise ValueError("Smooth-step limits cannot be negative.")
        field_unit_name, field_limit = UNIT_LIMITS[2]
        if not 0 <= config["amplitude"] <= field_limit:
            raise ValueError(f"Amplitude must be between 0 and {field_limit} {field_unit_name}.")
        if not 0 <= config["frequency"] <= 100:
            raise ValueError("Rotation frequency must be between 0 and 100 Hz.")
        if not 0 <= config["ramp"] <= 200:
            raise ValueError("Ramp must be between 0 and 200.")
        return config

    def _start_random_motion(self, _event: Any) -> None:
        if self.random_running:
            self._toast("Random Motion is already running.", error=True)
            return
        try:
            config = self._random_config_from_form()
            self._tx_delay_seconds()
        except ValueError as exc:
            self._toast(str(exc), error=True)
            return
        if self.serial_port is None:
            self._toast("Connect UART before starting Random Motion.", error=True)
            return
        if self.waiting_for_ready:
            self._toast("Wait for STM32 startup to complete before starting real Random Motion.", error=True)
            return
        if self.response_fault:
            self._toast("TX is blocked after a missing response. Reset SEQ before starting Random Motion.", error=True)
            return
        if config["enable_outputs"] and self._motor_operation_pending():
            self._toast("Random Motion cannot enable coils while a motor operation is running.", error=True)
            return
        if config["enable_outputs"]:
            self._confirm(
                "Start real Random Motion?",
                "Commands will be transmitted and all coil outputs will be enabled.",
                lambda: self._begin_random_motion(config),
            )
        else:
            self._begin_random_motion(config)

    def _begin_random_motion(self, config: dict[str, Any]) -> None:
        self.random_generation += 1
        generation = self.random_generation
        self.random_config = config
        now = time.monotonic()
        self.random_started_at = now
        self.random_ends_at = now + float(config["duration"])
        self.random_next_at = now + float(config["interval"])
        self.random_azimuth = (float(config["az_min"]) + float(config["az_max"])) / 2.0
        self.random_elevation = (float(config["el_min"]) + float(config["el_max"])) / 2.0
        self.random_running = True
        self.random_start_button.disabled = True
        self.random_pause_button.disabled = False
        self.random_progress.value = 0
        self._set_random_status("RUNNING", GREEN, GREEN_SOFT)
        self.dashboard_random_value.value = "Running"

        if self.input_unit != 2 and not self._send_random_command(0x00, bytes((2,)), "input unit = µT"):
            self._halt_random("ERROR", write_log=True)
            return
        if not self._random_step():
            self._halt_random("ERROR", write_log=True)
            return
        if config["enable_outputs"] and not self._send_random_command(0x07, bytes((3, 1)), "ALL coil outputs = RUN"):
            self._halt_random("ERROR", write_log=True)
            return
        if config["enable_outputs"]:
            self.coil_state_value.value = "Enable queued · awaiting response"
        self.log("INFO", "Random Motion started on hardware.")
        self.page.update()
        self.page.run_task(self._random_loop, generation)

    def _send_random_command(
        self,
        command_id: int,
        payload: bytes,
        description: str,
        enforce_delay: bool = True,
        bypass_gate: bool = False,
    ) -> bool:
        return self._send_command(
            command_id,
            payload,
            description,
            notify=False,
            enforce_delay=enforce_delay,
            source="random",
            bypass_gate=bypass_gate,
        )

    def _random_step(self) -> bool:
        config = self.random_config
        if config["mode"] == "random":
            azimuth = random.uniform(float(config["az_min"]), float(config["az_max"]))
            elevation = random.uniform(float(config["el_min"]), float(config["el_max"]))
        else:
            azimuth = self.random_azimuth + random.uniform(-float(config["step_az"]), float(config["step_az"]))
            elevation = self.random_elevation + random.uniform(-float(config["step_el"]), float(config["step_el"]))
            azimuth = min(float(config["az_max"]), max(float(config["az_min"]), azimuth))
            elevation = min(float(config["el_max"]), max(float(config["el_min"]), elevation))
        self.random_azimuth = float(round(azimuth))
        self.random_elevation = float(round(elevation))

        values = {
            "azimuth": int(self.random_azimuth),
            "elevation": int(self.random_elevation),
            "amplitude": int(round(float(config["amplitude"]))),
            "frequency": int(round(float(config["frequency"]))),
            "offset_x": 0,
            "offset_y": 0,
            "offset_z": 0,
            "ramp": int(round(float(config["ramp"]))),
        }
        try:
            payload = proto.encode_payload(proto.COMMAND_BY_ID[0x06], values)
        except Exception as exc:
            self.log("ERR", f"Random Motion parameters failed: {exc}")
            return False
        self.random_az_text.value = f"{self.random_azimuth:.0f}°"
        self.random_el_text.value = f"{self.random_elevation:+.0f}°"
        self.compass_arrow.rotate = ft.Rotate(angle=math.radians(self.random_azimuth))
        return self._send_random_command(
            0x06,
            payload,
            f"azimuth {values['azimuth']}°, elevation {values['elevation']}°",
        )

    async def _random_loop(self, generation: int) -> None:
        while self.random_running and generation == self.random_generation and not self.shutting_down:
            now = time.monotonic()
            remaining = max(0.0, self.random_ends_at - now)
            duration = float(self.random_config["duration"])
            self.random_remaining_text.value = self._format_random_time(remaining)
            self.random_progress.value = min(1.0, max(0.0, 1.0 - remaining / duration))
            if remaining <= 0:
                power_off = bool(self.random_config.get("power_off", True))
                self._halt_random("COMPLETED", write_log=False, update=False)
                self.random_progress.value = 1.0
                self.random_remaining_text.value = "00:00.0"
                self.random_next_text.value = "—"
                if power_off:
                    if self._send_random_command(0x07, bytes((3, 0)), "completed · ALL coil outputs = STOP"):
                        self.coil_state_value.value = "STOP queued · awaiting response"
                self.log("INFO", "Random Motion completed its configured duration.")
                self.page.update()
                return
            waiting_for_uart = self.awaiting_response is not None or bool(self.command_queue)
            if now >= self.random_next_at:
                if waiting_for_uart:
                    # Do not build a backlog. The next field is generated only
                    # after the preceding command chain has been acknowledged.
                    self.random_next_text.value = "waiting for UART response"
                elif not self._random_step():
                    self._halt_random("ERROR", write_log=True)
                    return
                self.random_next_at = now + float(self.random_config["interval"])
            if not waiting_for_uart:
                self.random_next_text.value = f"{max(0.0, self.random_next_at - now):.1f} s"
            try:
                self.page.update()
            except Exception:
                return
            await asyncio.sleep(0.05)

    @staticmethod
    def _format_random_time(seconds: float) -> str:
        tenths = max(0, int(math.ceil(seconds * 10.0)))
        minutes = tenths // 600
        remainder = (tenths % 600) / 10.0
        return f"{minutes:02d}:{remainder:04.1f}"

    def _set_random_status(self, text: str, color: str, background: str) -> None:
        self.random_status_text.value = text
        self.random_status_text.color = color
        self.random_status_box.bgcolor = background
        self._set_chip(
            self.random_header_chip,
            "Random " + text.lower(),
            color,
            background,
            ft.Icons.AUTO_AWESOME_ROUNDED,
        )

    def _halt_random(self, status: str, write_log: bool, update: bool = True) -> bool:
        was_running = self.random_running
        self.random_running = False
        self.random_generation += 1
        self.command_queue = [command for command in self.command_queue if command.source != "random"]
        self.random_start_button.disabled = False
        self.random_pause_button.disabled = True
        color = RED if status in ("ERROR", "EMERGENCY STOP", "DISCONNECTED") else AMBER
        background = RED_SOFT if color == RED else AMBER_SOFT
        self._set_random_status(status, color, background)
        self.dashboard_random_value.value = status.title()
        if was_running and write_log:
            self.log("INFO", f"Random Motion stopped · {status.lower()}.")
        if update and not self.shutting_down:
            self.page.update()
        return was_running

    def _stop_random_changes(self, _event: Any = None) -> None:
        self._halt_random("STOPPED", write_log=True)
        self.random_next_text.value = "—"
        self.page.update()

    def _random_emergency_stop(self, _event: Any = None) -> None:
        self._halt_random("EMERGENCY STOP", write_log=True, update=False)
        self._discard_queued_commands("by EMERGENCY STOP")
        self.random_next_text.value = "—"
        if self._send_random_command(
            0x07,
            bytes((3, 0)),
            "STOP RANDOM + ALL coil outputs = STOP",
            enforce_delay=False,
            bypass_gate=True,
        ):
            if self.serial_port is not None:
                self.coil_state_value.value = "Emergency STOP sent · awaiting response"
        self.page.update()

    # ------------------------------------------------------------------
    # Log, shutdown and CLI support
    # ------------------------------------------------------------------

    def log(self, tag: str, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {tag:<4} {message}"
        self.log_lines.append(line)
        color = {"TX": PRIMARY, "RX": GREEN, "ERR": RED, "SIM": AMBER}.get(tag, MUTED)
        self.log_view.controls.append(
            ft.Text(line, size=11, color=color, font_family=MONO, selectable=True)
        )
        if len(self.log_view.controls) > 1200:
            del self.log_view.controls[:200]
            del self.log_lines[:200]

        activity = ft.Row(
            [
                ft.Container(width=7, height=7, border_radius=4, bgcolor=color),
                ft.Text(f"[{stamp}] {tag}", size=10, color=MUTED, width=82, font_family=MONO),
                ft.Text(message.replace("\n", " ")[:92], size=11, color=TEXT, expand=True, no_wrap=True),
            ],
            spacing=8,
        )
        self.recent_activity.controls.insert(0, activity)
        del self.recent_activity.controls[4:]

    def _clear_log(self, _event: Any = None) -> None:
        self.log_lines.clear()
        self.log_view.controls.clear()
        self.page.update()

    def _on_page_close(self, _event: Any = None) -> None:
        self.shutting_down = True
        self._halt_random("STOPPED", write_log=False, update=False)
        self._disconnect(update=False)


def run_self_test() -> None:
    """Validate Flet-facing metadata plus the proven protocol implementation."""
    proto.run_self_test()
    assert proto.BAUDRATE == 115_200
    assert set(COMMAND_META) == set(range(0x19)) == set(proto.COMMAND_BY_ID)
    assert magnetometer_rate_error({"magnetometer": "0", "rate": "60"}) is None
    assert magnetometer_rate_error({"magnetometer": "1", "rate": "61"}) is not None
    assert magnetometer_rate_error({"magnetometer": "3", "rate": "250"}) is None
    mag_start_reply = bytes((0x97, 5, SUCCESS_STATUS, 0x00, 0x0C)) + struct.pack(">iii", 512, -16, 3) + b"\0\0\0\0"
    assert normal_response_fields(mag_start_reply) is not None
    assert "X 512 mG · Y -16 mG · Z 3 mG" in decode_response_en(mag_start_reply)
    assert ACTIVE_PROFILE == proto.PROFILE_CURRENT
    assert not BLOCKED_CURRENT_FIRMWARE
    assert all(code in STATUS_EN for code in COIL_STOP_STATUSES)
    assert SUCCESS_STATUS not in STARTUP_STATUSES
    assert startup_sensor(0x22) == ("Magnetometer 0", True)
    assert startup_sensor(0x21) == ("Magnetometer 3", False)
    assert startup_sensor(0x26) == ("Thermometer 0", False)
    assert startup_sensor(0x2D) == ("Thermometer 3", True)
    assert len({startup_sensor(status)[0] for status in STARTUP_STATUSES}) == STARTUP_SENSOR_COUNT
    long_move =QueuedCommand(0x0F, 1, struct.pack(">BBH", 0, 0, 5000), b"", "move")
    assert long_move.response_timeout > 5000 * MOTOR_STEP_TIME_S
    exchange_reset = QueuedCommand(0x0E, 2, bytes((0,)), b"", "reference")
    assert exchange_reset.response_timeout > EXCHANGE_MOTOR_RESET_STEPS * MOTOR_STEP_TIME_S
    assert QueuedCommand(0x0F, 3, struct.pack(">BBH", 0, 0, 10), b"", "move").response_timeout == STANDARD_RESPONSE_TIMEOUT
    assert QueuedCommand(0x0D, 4, b"", b"", "ping").response_timeout == STANDARD_RESPONSE_TIMEOUT

    assert unit_limit_error(0x06, {"amplitude": "6000", "offset_x": "-6000"}, 2) is None
    assert unit_limit_error(0x06, {"amplitude": "6001"}, 2) is not None
    assert unit_limit_error(0x02, {"offset_z": "-1001"}, 1) is not None
    assert unit_limit_error(0x04, {"amplitude_y": "12000", "offset_y": "12000"}, 0) is None
    assert unit_limit_error(0x01, {"amplitude": "12001"}, 0) is not None
    assert unit_limit_error(0x07, {"output": "3", "state": "1"}, 1) is None

    motor_payload = struct.pack(">BBH", 1, 1, 1000)
    assert len(motor_payload) == 4
    assert motor_payload == b"\x01\x01\x03\xE8"
    motor_raw, motor_wire = proto.build_command(0x0F, 7, motor_payload, ACTIVE_PROFILE)
    assert int.from_bytes(motor_raw[2:4], "big") == 4
    assert proto.FrameStreamParser().feed(motor_wire) == [motor_raw]

    random_payload = proto.encode_payload(proto.COMMAND_BY_ID[0x06], {
        "azimuth": 180, "elevation": -30, "amplitude": 100, "frequency": 1,
        "offset_x": 0, "offset_y": 0, "offset_z": 0, "ramp": 10,
    })
    assert len(random_payload) == 15
    random_raw, random_wire = proto.build_command(0x06, 8, random_payload, ACTIVE_PROFILE)
    assert int.from_bytes(random_raw[2:4], "big") == 15
    assert proto.FrameStreamParser().feed(random_wire) == [random_raw]

    rotating_response = bytes((0x86, 8, SUCCESS_STATUS, 0, 0)) + b"\0\0\0\0"
    assert normal_response_fields(rotating_response) == (0x86, 8, SUCCESS_STATUS, b"")
    thermometer_notification = bytes((0x9C, SUCCESS_STATUS, 0, 3, 1, 0x01, 0x90)) + b"\0\0\0\0"
    assert notification_fields(thermometer_notification) == (0x9C, SUCCESS_STATUS, b"\x01\x01\x90")
    assert normal_response_fields(thermometer_notification) is None
    assert "25.00 °C" in decode_response_en(thermometer_notification)
    magnetometer_notification = bytes((0x9D, SUCCESS_STATUS, 0, 13, 2)) + struct.pack(">iii", 1200, -350, 7) + b"\0\0\0\0"
    assert normal_response_fields(magnetometer_notification) is None
    assert "X 1200 mG · Y -350 mG · Z 7 mG" in decode_response_en(magnetometer_notification)
    magnetometer_temperature = bytes((0x96, 4, SUCCESS_STATUS, 0, 1, 100)) + b"\0\0\0\0"
    assert notification_fields(magnetometer_temperature) is None
    assert "5.0 °C" in decode_response_en(magnetometer_temperature)
    output_response = bytes((0x87, 0, SUCCESS_STATUS, 0, 0, 0xF1, 0xA8, 0xD8, 0xE4))
    assert normal_response_fields(output_response) == (0x87, 0, SUCCESS_STATUS, b"")
    assert EXPECTED_RESPONSE_IDS[0x07] == 0x87
    hello_response = bytes((0x8D, 3, SUCCESS_STATUS, 0x00, 0x05)) + b"Hello" + b"\0\0\0\0"
    assert normal_response_fields(hello_response) == (0x8D, 3, SUCCESS_STATUS, b"Hello")

    split_parser = proto.FrameStreamParser()
    assert split_parser.feed(b"\x7E") == []
    assert split_parser.feed(b"\x7E") == []
    assert split_parser.feed(rotating_response + b"\x7E") == [rotating_response]
    print(
        "Flet UI self-test OK: any-response gating metadata, fragmented framing, "
        "motor interlock IDs and Random Motion frames."
    )


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="NanoRobot Control Center · Flet desktop GUI")
    parser.add_argument("--self-test", action="store_true", help="Validate protocol payloads without opening the GUI.")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.self_test:
        run_self_test()
        return 0
    if ft is None:
        message = f"Flet is not installed. Install the requirements with:\n{INSTALL_COMMAND}\nthen start the application again."
        print(message, file=sys.stderr)
        try:
            import ctypes
            ctypes.windll.user32.MessageBoxW(0, message, "NanoRobot Control Center", 0x10)
        except Exception:
            pass
        return 1
    if hasattr(ft, "run"):
        ft.run(lambda page: NanoRobotApp(page))
    else:
        ft.app(target=lambda page: NanoRobotApp(page))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
