#!/usr/bin/env python3
"""NanoRobot Control Center - desktop UART controller for commands 0x00..0x16.

The STM32 firmware currently compiles its incoming CRC check out with ``#if 0``.
Commands therefore contain four zero CRC bytes.  Responses are displayed but
their CRC is intentionally not validated.
"""

from __future__ import annotations

import argparse
import math
import queue
import random
import struct
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Any, Iterable

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:  # The GUI can still open and explain how to install it.
    serial = None
    list_ports = None


FRAME_DELIMITER = 0x7E
ESCAPE_CHARACTER = 0x7D
CRC_PLACEHOLDER = b"\x00\x00\x00\x00"
BAUDRATE = 115_200

PROFILE_CURRENT = "Aktuální firmware · kompatibilní délky"
PROFILE_FIXED = "Opravený firmware · skutečné délky"

FIRMWARE_PAYLOAD_LENGTHS = {
    0x00: 1, 0x01: 10, 0x02: 27, 0x03: 9, 0x04: 25, 0x05: 1, 0x06: 15,
    0x07: 2, 0x08: 0, 0x09: 18, 0x0A: 3, 0x0B: 3, 0x0C: 19,
    0x0D: 0, 0x0E: 1, 0x0F: 4, 0x10: 1, 0x11: 1, 0x12: 1, 0x13: 2,
    0x14: 3, 0x15: 1, 0x16: 1, 0x17: 2, 0x18: 1,
}

# Tyto dva rámce se ani bez escapování nevejdou do 32B RX bufferu aktuálního
# firmware. V GUI zůstávají dostupné pro opravený firmware.
CURRENT_FIRMWARE_OVERSIZE: set[int] = set()
CURRENT_FIRMWARE_LENGTH_CONFLICT: set[int] = set()


@dataclass(frozen=True)
class FieldSpec:
    key: str
    label: str
    pack: str
    default: int | float
    minimum: int | float
    maximum: int | float
    unit: str = ""
    choices: tuple[tuple[str, int], ...] = ()
    hint: str = ""


@dataclass(frozen=True)
class CommandSpec:
    command_id: int
    title: str
    summary: str
    fields: tuple[FieldSpec, ...]
    category: str = "Cívky"
    caution: str = ""


AXES = (("X", 0), ("Y", 1), ("Z", 2))
INPUT_TYPES = (("mV", 0), ("mA", 1), ("µT", 2))
OUTPUTS = (("X", 0), ("Y", 1), ("Z", 2), ("Všechny", 3), ("X1", 4), ("X2", 5))
STATES = (("STOP / vypnout", 0), ("RUN / zapnout", 1))
MOTORS = (("Výměnný motor", 0), ("Focus motor", 1))
DIRECTIONS = (("Nahoru / vpřed", 0), ("Dolů / vzad", 1))
SAMPLES = tuple((f"Vzorek {index:02d}", index) for index in range(20))
ACTIONS = (("Vypnout", 0), ("Zapnout", 1))
LIGHTS = (("Podsvit 0", 0), ("Podsvit 1", 1), ("UV Bio-killer", 2))
SENSORS = tuple((f"Senzor {index}", index) for index in range(4))
MAGNETOMETERS = (
    ("Ambient magnetometer 0", 0),
    ("Ambient magnetometer 1", 1),
    ("Coil magnetometer 0", 2),
    ("Coil magnetometer 1", 3),
)


def f(
    key: str,
    label: str,
    pack: str,
    default: int | float,
    minimum: int | float,
    maximum: int | float,
    unit: str = "",
    choices: tuple[tuple[str, int], ...] = (),
    hint: str = "",
) -> FieldSpec:
    return FieldSpec(key, label, pack, default, minimum, maximum, unit, choices, hint)


COMMANDS: tuple[CommandSpec, ...] = (
    CommandSpec(0x00, "Jednotky vstupu", "Volba jednotek amplitudy a offsetu.", (
        f("input_type", "Jednotka", "B", 0, 0, 2, choices=INPUT_TYPES),
    )),
    CommandSpec(0x01, "Asynchronní – jeden kanál", "Nastavení jednoho nezávislého průběhu.", (
        f("axis", "Osa", "B", 0, 0, 2, choices=AXES),
        f("amplitude", "Amplituda", "H", 0, 0, 12000, hint="Rozsah závisí na zvolené jednotce."),
        f("frequency", "Frekvence", "B", 1, 0, 100, "Hz"),
        f("offset", "Offset", "h", 0, -12000, 12000),
        f("phase", "Fáze", "H", 0, 0, 360, "°"),
        f("ramp", "Rampa", "H", 0, 0, 200, "0,1 s"),
    )),
    CommandSpec(0x02, "Asynchronní – všechny kanály", "Nezávislé nastavení X, Y a Z.", (
        f("amplitude_x", "Amplituda X", "H", 0, 0, 12000),
        f("amplitude_y", "Amplituda Y", "H", 0, 0, 12000),
        f("amplitude_z", "Amplituda Z", "H", 0, 0, 12000),
        f("frequency_x", "Frekvence X", "B", 1, 0, 100, "Hz"),
        f("frequency_y", "Frekvence Y", "B", 1, 0, 100, "Hz"),
        f("frequency_z", "Frekvence Z", "B", 1, 0, 100, "Hz"),
        f("offset_x", "Offset X", "h", 0, -12000, 12000),
        f("offset_y", "Offset Y", "h", 0, -12000, 12000),
        f("offset_z", "Offset Z", "h", 0, -12000, 12000),
        f("phase_x", "Fáze X", "H", 0, 0, 360, "°"),
        f("phase_y", "Fáze Y", "H", 0, 0, 360, "°"),
        f("phase_z", "Fáze Z", "H", 0, 0, 360, "°"),
        f("ramp_x", "Rampa X", "H", 0, 0, 200, "0,1 s"),
        f("ramp_y", "Rampa Y", "H", 0, 0, 200, "0,1 s"),
        f("ramp_z", "Rampa Z", "H", 0, 0, 200, "0,1 s"),
    )),
    CommandSpec(0x03, "Synchronní – jeden kanál", "Jeden kanál se společnou synchronní frekvencí.", (
        f("axis", "Osa", "B", 0, 0, 2, choices=AXES),
        f("amplitude", "Amplituda", "H", 0, 0, 12000),
        f("offset", "Offset", "h", 0, -12000, 12000),
        f("phase", "Fáze", "H", 0, 0, 360, "°"),
        f("ramp", "Rampa", "H", 0, 0, 200, "0,1 s"),
    )),
    CommandSpec(0x04, "Synchronní – všechny kanály", "Společná frekvence, samostatné amplitudy a fáze.", (
        f("amplitude_x", "Amplituda X", "H", 0, 0, 12000),
        f("amplitude_y", "Amplituda Y", "H", 0, 0, 12000),
        f("amplitude_z", "Amplituda Z", "H", 0, 0, 12000),
        f("frequency", "Společná frekvence", "B", 1, 0, 100, "Hz"),
        f("offset_x", "Offset X", "h", 0, -12000, 12000),
        f("offset_y", "Offset Y", "h", 0, -12000, 12000),
        f("offset_z", "Offset Z", "h", 0, -12000, 12000),
        f("phase_x", "Fáze X", "H", 0, 0, 360, "°"),
        f("phase_y", "Fáze Y", "H", 0, 0, 360, "°"),
        f("phase_z", "Fáze Z", "H", 0, 0, 360, "°"),
        f("ramp_x", "Rampa X", "H", 0, 0, 200, "0,1 s"),
        f("ramp_y", "Rampa Y", "H", 0, 0, 200, "0,1 s"),
        f("ramp_z", "Rampa Z", "H", 0, 0, 200, "0,1 s"),
    )),
    CommandSpec(0x05, "Synchronní frekvence", "Změna společné synchronní frekvence.", (
        f("frequency", "Frekvence", "B", 1, 0, 100, "Hz"),
    )),
    CommandSpec(0x06, "Rotující magnetické pole", "Výsledné pole definované azimutem a elevací.", (
        f("azimuth", "Azimut", "H", 0, 0, 360, "°"),
        f("elevation", "Elevace", "h", 0, -180, 180, "°"),
        f("amplitude", "Amplituda pole", "H", 0, 0, 12000),
        f("frequency", "Frekvence rotace", "B", 1, 0, 100, "Hz"),
        f("offset_x", "Offset X", "h", 0, -12000, 12000),
        f("offset_y", "Offset Y", "h", 0, -12000, 12000),
        f("offset_z", "Offset Z", "h", 0, -12000, 12000),
        f("ramp", "Rampa", "H", 0, 0, 200, "0,1 s"),
    )),
    CommandSpec(0x07, "Výstupy", "Povolení nebo zakázání výstupů cívek.", (
        f("output", "Výstup", "B", 3, 0, 5, choices=OUTPUTS),
        f("state", "Stav", "B", 0, 0, 1, choices=STATES),
    )),
    CommandSpec(0x08, "Synchronizace kanálů", "Restart časové základny asynchronních kanálů.", ()),
    CommandSpec(0x09, "Kalibrace pole → proud", "Matice 3×3 pro převod požadovaného pole na proud.", (
        f("xx", "XX", "h", 128, -1000, 1000), f("xy", "XY", "h", 0, -1000, 1000),
        f("xz", "XZ", "h", 0, -1000, 1000), f("yx", "YX", "h", 0, -1000, 1000),
        f("yy", "YY", "h", 138, -1000, 1000), f("yz", "YZ", "h", 0, -1000, 1000),
        f("zx", "ZX", "h", 0, -1000, 1000), f("zy", "ZY", "h", 0, -1000, 1000),
        f("zz", "ZZ", "h", 113, -1000, 1000),
    )),
    CommandSpec(0x0A, "Nula proudového senzoru", "Nastavení napětí senzoru odpovídajícího 0 A.", (
        f("axis", "Osa", "B", 0, 0, 2, choices=AXES),
        f("zero", "Nulové napětí", "h", 1650, -5000, 5000, "mV"),
    )),
    CommandSpec(0x0B, "Filtr proudového senzoru", "IIR alpha, jeden bit odpovídá 0,01.", (
        f("axis", "Osa", "B", 0, 0, 2, choices=AXES),
        f("iir", "IIR alpha", "H", 20, 0, 100, "×0,01", hint="Pro stabilní filtr držte 0,00–1,00."),
    )),
    CommandSpec(0x0C, "PI regulátor", "Nastavení PI regulátoru vybraného proudového kanálu.", (
        f("axis", "Osa", "B", 0, 0, 2, choices=AXES),
        f("kp", "Kp", "d", 1.0, -1_000_000.0, 1_000_000.0),
        f("ki", "Ki", "d", 0.1, -1_000_000.0, 1_000_000.0),
        f("low", "Dolní limit", "b", -120, -120, 120, "×0,1 V"),
        f("high", "Horní limit", "b", 120, -120, 120, "×0,1 V"),
    )),
    CommandSpec(0x0D, "Kontrola spojení", "Odpověď obsahuje text Hello.", (), category="Komunikace"),
    CommandSpec(0x0E, "Referencování motoru", "Pohyb na mechanický doraz a nastavení výchozí polohy.", (
        f("motor", "Motor", "B", 0, 0, 1, choices=MOTORS),
    ), category="Motory", caution="Pozor: firmware nemá koncový spínač a jede pevný počet kroků do dorazu."),
    CommandSpec(0x0F, "Pohyb motoru", "Relativní pohyb krokového motoru.", (
        f("motor", "Motor", "B", 0, 0, 1, choices=MOTORS),
        f("direction", "Směr", "B", 0, 0, 1, choices=DIRECTIONS),
        f("steps", "Počet kroků", "H", 10, 0, 5000, "kroků"),
    ), category="Motory"),
    CommandSpec(0x10, "Přejít na vzorek", "Pohyb výměnného motoru na uloženou pozici vzorku.", (
        f("sample", "Vzorek", "B", 0, 0, 19, choices=SAMPLES),
    ), category="Motory"),
    CommandSpec(0x11, "Uložit pozici vzorku", "Uloží aktuální polohu výměnného motoru.", (
        f("sample", "Vzorek", "B", 0, 0, 19, choices=SAMPLES),
    ), category="Motory", caution="Pozice je v této verzi pouze v RAM a po resetu se obnoví výchozí tabulka."),
    CommandSpec(0x12, "Ventilátor", "Zapnutí nebo vypnutí ventilátoru.", (
        f("action", "Stav", "B", 0, 0, 1, choices=ACTIONS),
    ), category="Periferie"),
    CommandSpec(0x13, "Osvětlení", "Ovládání podsvitu nebo UV světla.", (
        f("light", "Světlo", "B", 0, 0, 2, choices=LIGHTS),
        f("action", "Stav", "B", 0, 0, 1, choices=ACTIONS),
    ), category="Periferie", caution="UV výstup zapínejte pouze v bezpečných podmínkách."),
    CommandSpec(0x14, "Spustit teploměr", "Perioda 0 provede jednorázové měření; vyšší hodnota spustí notifikace.", (
        f("sensor", "Teploměr", "B", 0, 0, 3, choices=SENSORS),
        f("period", "Perioda", "H", 0, 0, 65535, "smyček po 1 s"),
    ), category="Senzory"),
    CommandSpec(0x15, "Zastavit teploměr", "Ukončí kontinuální měření vybraného teploměru.", (
        f("sensor", "Teploměr", "B", 0, 0, 3, choices=SENSORS),
    ), category="Senzory"),
    CommandSpec(0x16, "Teplota magnetometru", "Jednorázové čtení teploty magnetometru.", (
        f("sensor", "Magnetometr", "B", 0, 0, 3, choices=SENSORS),
    ), category="Senzory"),
    CommandSpec(0x17, "Start magnetometer", "Single field measurement (rate 0) or continuous notifications.", (
        f("magnetometer", "Magnetometer", "B", 0, 0, 3, choices=MAGNETOMETERS),
        f("rate", "Rate", "B", 0, 0, 250,
          hint="Ambient: period 1-60 s. Coil: frequency 1-250 Hz. 0 = single measurement."),
    ), category="Senzory"),
    CommandSpec(0x18, "Stop magnetometer", "Stop continuous magnetic field notifications.", (
        f("magnetometer", "Magnetometer", "B", 0, 0, 3, choices=MAGNETOMETERS),
    ), category="Senzory"),
)

COMMAND_BY_ID = {command.command_id: command for command in COMMANDS}


def _parse_number(value: Any, floating: bool) -> int | float:
    if isinstance(value, (int, float)):
        return float(value) if floating else int(value)
    text = str(value).strip().replace(",", ".")
    if floating:
        return float(text)
    return int(text, 0)


def encode_payload(spec: CommandSpec, values: dict[str, Any]) -> bytes:
    payload = bytearray()
    for field in spec.fields:
        floating = field.pack in ("f", "d")
        parsed = _parse_number(values[field.key], floating)
        if not field.minimum <= parsed <= field.maximum:
            raise ValueError(
                f"{field.label}: hodnota {parsed} je mimo rozsah "
                f"{field.minimum}…{field.maximum}."
            )
        payload.extend(struct.pack(">" + field.pack, parsed))
    return bytes(payload)


def escape_body(body: bytes) -> bytes:
    encoded = bytearray((FRAME_DELIMITER,))
    for byte in body:
        if byte in (FRAME_DELIMITER, ESCAPE_CHARACTER):
            encoded.append(ESCAPE_CHARACTER)
        encoded.append(byte)
    encoded.append(FRAME_DELIMITER)
    return bytes(encoded)


def build_command(
    command_id: int,
    sequence: int,
    payload: bytes,
    profile: str = PROFILE_CURRENT,
) -> tuple[bytes, bytes]:
    if not 0 <= sequence <= 0xFF:
        raise ValueError("Sekvence musí být 0…255.")
    wire_payload = payload
    if profile == PROFILE_CURRENT:
        expected = FIRMWARE_PAYLOAD_LENGTHS[command_id]
        wire_payload = payload[:expected].ljust(expected, b"\x00")
    raw = (
        bytes((command_id, sequence))
        + len(wire_payload).to_bytes(2, "big")
        + wire_payload
        + CRC_PLACEHOLDER
    )
    return raw, escape_body(raw)


def hex_bytes(data: bytes, columns: int = 16) -> str:
    groups = [f"{byte:02X}" for byte in data]
    return "\n".join(" ".join(groups[i:i + columns]) for i in range(0, len(groups), columns))


def _crc32_mpeg2_table() -> tuple[int, ...]:
    table = []
    for index in range(256):
        crc = index << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else (crc << 1)
            crc &= 0xFFFFFFFF
        table.append(crc)
    return tuple(table)


CRC32_MPEG2_TABLE = _crc32_mpeg2_table()


def crc32_mpeg2(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFFFF) ^ CRC32_MPEG2_TABLE[(crc >> 24) ^ byte]
    return crc


def frame_crc_ok(body: bytes) -> bool:
    return len(body) > 4 and int.from_bytes(body[-4:], "big") == crc32_mpeg2(body[:-4])


def tmp100_celsius(raw: int) -> float:
    value = raw & 0x0FFF
    if value & 0x0800:
        value -= 0x1000
    return value * 0.0625


def mmc5603_celsius(raw: int) -> float:
    return raw * 0.8 - 75.0


class FrameStreamParser:
    """Streaming parser for the firmware's 0x7D + unchanged-byte escaping."""

    def __init__(self) -> None:
        self.in_frame = False
        self.escaped = False
        self.body = bytearray()
        self.outside = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        frames: list[bytes] = []
        for byte in data:
            if not self.in_frame:
                if byte == FRAME_DELIMITER:
                    self.in_frame = True
                    self.escaped = False
                    self.body.clear()
                else:
                    # Firmware po self-testu posílá neorámovaný text
                    # "STM32 ready!!!". Uchováme jej odděleně od rámců.
                    self.outside.append(byte)
                    if len(self.outside) > 2048:
                        del self.outside[:-1024]
                continue
            if self.escaped:
                self.body.append(byte)
                self.escaped = False
            elif byte == ESCAPE_CHARACTER:
                self.escaped = True
            elif byte == FRAME_DELIMITER:
                if self.body:
                    frames.append(bytes(self.body))
                    self.in_frame = False
                    self.body.clear()
                else:
                    # Consecutive delimiters can appear between firmware
                    # messages (or be split across separate USB reads). They
                    # are boundaries/idle bytes, never a valid empty frame.
                    # Keep the latest delimiter as a possible frame start.
                    self.in_frame = True
                    self.escaped = False
            else:
                self.body.append(byte)
        return frames

    def drain_outside(self) -> bytes:
        data = bytes(self.outside)
        self.outside.clear()
        return data


STATUS_NAMES = {
    0x01: "Vstupní fronta plná",
    0x02: "Vstupní fronta prázdná",
    0x03: "UART chyba",
    0x04: "UART TX chyba",
    0x05: "UART TX zaneprázdněn",
    0x06: "UART timeout",
    0x07: "Nenalezen začátek rámce",
    0x08: "Nenalezen konec rámce",
    0x09: "Neznámý příkaz",
    0x0A: "Nesprávná délka payloadu",
    0x0B: "Nesprávné CRC",
    0x0C: "Nesprávná délka rámce",
    0x0E: "NULL ukazatel",
    0x11: "L99SM81V SPI chyba",
    0x12: "STM32 SPI zaneprázdněno",
    0x13: "Neinicializováno",
    0x14: "Nesprávný parametr",
    0x15: "Krokový motor vypnut",
    0x1B: "L99SM81V pravděpodobně vypnut",
    0x1C: "STM32 I²C chyba",
    0x1D: "STM32 I²C zaneprázdněno",
    0x1E: "Magnetometr 0: CHYBA",
    0x1F: "Magnetometr 1: CHYBA",
    0x20: "Magnetometr 2: CHYBA",
    0x21: "Magnetometr 3: CHYBA",
    0x22: "Magnetometr 0: OK",
    0x23: "Magnetometr 1: OK",
    0x24: "Magnetometr 2: OK",
    0x25: "Magnetometr 3: OK",
    0x26: "Teploměr 0: CHYBA",
    0x27: "Teploměr 1: CHYBA",
    0x28: "Teploměr 2: CHYBA",
    0x29: "Teploměr 3: CHYBA",
    0x2A: "Teploměr 0: OK",
    0x2B: "Teploměr 1: OK",
    0x2C: "Teploměr 2: OK",
    0x2D: "Teploměr 3: OK",
    0x2F: "Cívky neinicializovány",
    0x30: "Neplatný typ vstupu",
    0x31: "Neplatný kanál",
    0x32: "Neplatný výstup",
    0x33: "Neplatný stav výstupu",
    0x35: "Neplatné PI zesílení",
    0x36: "Neplatné PI limity",
    0x77: "SUCCESS (interní kód firmware)",
}


def identify_response(response_id: int) -> str:
    if response_id == 0xFF:
        return "Systémová notifikace"
    if response_id == 0x9C:
        return "Thermometer notification"
    if response_id == 0x9D:
        return "Magnetometer notification"
    command_id: int | None = response_id - 0x80 if 0x80 <= response_id <= 0x98 else None
    if command_id is not None and command_id in COMMAND_BY_ID:
        return COMMAND_BY_ID[command_id].title
    return f"Neznámá odpověď 0x{response_id:02X}"


def decode_response(body: bytes) -> str:
    if len(body) < 4:
        return f"Krátký/neúplný rámec ({len(body)} B): {hex_bytes(body)}"
    if body[0] in (0x9C, 0x9D, 0xFF):
        response_id, status = body[0], body[1]
        payload_length = int.from_bytes(body[2:4], "big")
        payload = body[4:4 + payload_length]
        status_name = STATUS_NAMES.get(status, f"0x{status:02X}")
        details = [f"{identify_response(response_id)} | ID 0x{response_id:02X}", f"Status: {status_name}"]
        if response_id == 0x9C and len(payload) >= 3:
            temp_raw = int.from_bytes(payload[1:3], "big")
            details.append(f"Thermometer {payload[0]} · {tmp100_celsius(temp_raw):.2f} °C · raw 0x{temp_raw:04X}")
        elif response_id == 0x9D and len(payload) >= 13:
            mag_x, mag_y, mag_z = struct.unpack(">iii", payload[1:13])
            details.append(f"Magnetometer {payload[0]} · X {mag_x} mG · Y {mag_y} mG · Z {mag_z} mG")
        elif payload:
            details.append(f"Data: {hex_bytes(payload)}")
        return "\n".join(details)
    if len(body) < 5:
        return f"Krátký/neúplný rámec ({len(body)} B): {hex_bytes(body)}"
    response_id, sequence, status = body[0], body[1], body[2]
    payload_length = int.from_bytes(body[3:5], "big")
    payload_end = min(5 + payload_length, len(body))
    payload = body[5:payload_end]
    crc = body[payload_end:payload_end + 4]
    status_name = STATUS_NAMES.get(status, f"0x{status:02X}")
    result = [
        f"{identify_response(response_id)} | ID 0x{response_id:02X} | SEQ {sequence}",
        f"Status: {status_name} | Payload: {payload_length} B",
    ]
    if payload:
        if response_id == 0x8D:
            result.append(f"Data: {payload.decode('ascii', errors='replace')}")
        elif response_id == 0x94 and len(payload) >= 2:
            temp_raw = int.from_bytes(payload[:2], "big")
            result.append(f"Temperature {tmp100_celsius(temp_raw):.2f} °C · raw 0x{temp_raw:04X}")
        elif response_id == 0x96:
            result.append(f"Magnetometer temperature {mmc5603_celsius(payload[0]):.1f} °C · raw {payload[0]}")
        elif response_id == 0x97 and len(payload) >= 12:
            mag_x, mag_y, mag_z = struct.unpack(">iii", payload[:12])
            result.append(f"Magnetic field · X {mag_x} mG · Y {mag_y} mG · Z {mag_z} mG")
        else:
            result.append(f"Data: {hex_bytes(payload)}")
    if crc:
        result.append(f"CRC (nekontrolováno): {hex_bytes(crc)}")
    if 5 + payload_length + 4 != len(body):
        result.append(f"Varování: délka rámce nesedí ({len(body)} B přijato).")
    return "\n".join(result)


class CoilConsole(tk.Tk):
    BG = "#F5F3FF"
    SURFACE = "#FFFFFF"
    CARD = "#FFFFFF"
    BORDER = "#DDD6FE"
    TEXT = "#2E2442"
    MUTED = "#756A8A"
    ACCENT = "#7C3AED"
    ACCENT_DARK = "#6D28D9"
    GREEN = "#059669"
    RED = "#DC2626"
    ORANGE = "#D97706"

    def __init__(self) -> None:
        super().__init__()
        self.title("NanoRobot Control Center")
        self.geometry("1440x940")
        self.minsize(1180, 760)
        self.configure(bg=self.BG)

        self.serial_port: Any = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()
        self.rx_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.frame_parser = FrameStreamParser()
        self.ascii_rx_buffer = bytearray()
        self.waiting_for_ready = False
        self.startup_wait_deadline = 0.0
        self.pending_tx: list[tuple[bytes, str]] = []
        self.field_vars: dict[str, tk.StringVar] = {}
        self.current_command = COMMANDS[0]
        self.visible_command_indices = list(range(len(COMMANDS)))

        # Random Motion runtime state.
        self.random_running = False
        self.random_after_id: str | None = None
        self.random_started_at = 0.0
        self.random_ends_at = 0.0
        self.random_next_change_at = 0.0
        self.random_config: dict[str, float | str | bool] = {}
        self.random_current_az_value = 0.0
        self.random_current_el_value = 0.0

        self.port_var = tk.StringVar()
        self.profile_var = tk.StringVar(value=PROFILE_CURRENT)
        self.sequence_var = tk.StringVar(value="0")
        self.auto_sequence_var = tk.BooleanVar(value=True)
        self.connection_var = tk.StringVar(value="●  Odpojeno")
        self.category_var = tk.StringVar(value="Vše")
        self.preview_var = tk.StringVar()
        self.random_mode_var = tk.StringVar(value="Plynulé kroky")
        self.random_duration_var = tk.StringVar(value="60")
        self.random_interval_var = tk.StringVar(value="1.0")
        self.random_az_min_var = tk.StringVar(value="0")
        self.random_az_max_var = tk.StringVar(value="360")
        self.random_el_min_var = tk.StringVar(value="-90")
        self.random_el_max_var = tk.StringVar(value="90")
        self.random_step_az_var = tk.StringVar(value="20")
        self.random_step_el_var = tk.StringVar(value="10")
        self.random_amplitude_var = tk.StringVar(value="100")
        self.random_frequency_var = tk.StringVar(value="1")
        self.random_ramp_var = tk.StringVar(value="10")
        self.random_simulation_var = tk.BooleanVar(value=True)
        self.random_enable_outputs_var = tk.BooleanVar(value=True)
        self.random_power_off_finish_var = tk.BooleanVar(value=True)
        self.random_status_var = tk.StringVar(value="● PŘIPRAVENO")
        self.random_remaining_var = tk.StringVar(value="01:00.0")
        self.random_next_var = tk.StringVar(value="—")
        self.random_current_az_var = tk.StringVar(value="—")
        self.random_current_el_var = tk.StringVar(value="—")
        self.random_progress_var = tk.DoubleVar(value=0.0)

        self._configure_style()
        self._build_ui()
        self.refresh_ports()
        self.show_command(0)
        self.after(50, self._poll_serial)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background=self.BG, foreground=self.TEXT, font=("Segoe UI", 10))
        style.configure("Card.TFrame", background=self.CARD, relief="flat")
        style.configure("Surface.TFrame", background=self.SURFACE)
        style.configure("Header.TLabel", background=self.BG, foreground=self.TEXT, font=("Segoe UI Semibold", 23))
        style.configure("Sub.TLabel", background=self.BG, foreground=self.MUTED, font=("Segoe UI", 10))
        style.configure("CardTitle.TLabel", background=self.CARD, foreground=self.TEXT, font=("Segoe UI Semibold", 12))
        style.configure("Card.TLabel", background=self.CARD, foreground=self.TEXT)
        style.configure("Hint.TLabel", background=self.CARD, foreground=self.MUTED, font=("Segoe UI", 9))
        style.configure("Warn.TLabel", background=self.CARD, foreground=self.ORANGE, font=("Segoe UI Semibold", 9))
        style.configure("TEntry", fieldbackground="#FAF9FF", foreground=self.TEXT, insertcolor=self.TEXT, bordercolor=self.BORDER)
        style.configure("TCombobox", fieldbackground="#FAF9FF", foreground=self.TEXT, arrowcolor=self.ACCENT, bordercolor=self.BORDER)
        style.map("TCombobox", fieldbackground=[("readonly", "#FAF9FF")], foreground=[("readonly", self.TEXT)])
        style.configure("TCheckbutton", background=self.CARD, foreground=self.TEXT)
        style.map("TCheckbutton", background=[("active", self.CARD)])
        style.configure("Accent.TButton", background=self.ACCENT, foreground="#FFFFFF", font=("Segoe UI Semibold", 10), padding=(16, 9))
        style.map("Accent.TButton", background=[("active", self.ACCENT_DARK), ("disabled", self.BORDER)])
        style.configure("Secondary.TButton", background="#EDE9FE", foreground=self.ACCENT_DARK, padding=(12, 8))
        style.map("Secondary.TButton", background=[("active", "#DDD6FE")])
        style.configure("Danger.TButton", background=self.RED, foreground="#FFFFFF", font=("Segoe UI Semibold", 10), padding=(14, 9))
        style.map("Danger.TButton", background=[("active", "#B91C1C")])
        style.configure("Vertical.TScrollbar", background=self.CARD, troughcolor=self.SURFACE, arrowcolor=self.MUTED)
        style.configure("TNotebook", background=self.BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
        style.configure(
            "TNotebook.Tab",
            background="#EDE9FE",
            foreground=self.MUTED,
            font=("Segoe UI Semibold", 10),
            padding=(20, 10),
            borderwidth=0,
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", self.CARD), ("active", "#DDD6FE")],
            foreground=[("selected", self.ACCENT_DARK), ("active", self.ACCENT_DARK)],
        )
        style.configure("Purple.Horizontal.TProgressbar", troughcolor="#EDE9FE", background=self.ACCENT, borderwidth=0)
        style.configure("Metric.TLabel", background=self.CARD, foreground=self.ACCENT_DARK, font=("Segoe UI Semibold", 18))
        style.configure("Section.TLabel", background=self.CARD, foreground=self.TEXT, font=("Segoe UI Semibold", 11))

    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(2, weight=4)
        self.grid_rowconfigure(3, weight=1)

        header = ttk.Frame(self, style="Surface.TFrame", padding=(24, 18))
        header.grid(row=0, column=0, sticky="ew")
        header.grid_columnconfigure(0, weight=1)
        ttk.Label(header, text="NanoRobot Control Center", style="Header.TLabel", background=self.SURFACE).grid(row=0, column=0, sticky="w")
        ttk.Label(
            header,
            text=f"Řízení cívek, krokových motorů a periferií • UART4 PC10/PC11 • {BAUDRATE} 8N1",
            foreground=self.MUTED,
            background=self.SURFACE,
        ).grid(row=1, column=0, sticky="w", pady=(3, 0))
        header_status = ttk.Frame(header, style="Surface.TFrame")
        header_status.grid(row=0, column=1, rowspan=2, padx=(20, 0), sticky="e")
        self.connection_label = ttk.Label(
            header_status,
            textvariable=self.connection_var,
            foreground=self.RED,
            background=self.SURFACE,
            font=("Segoe UI Semibold", 11),
        )
        self.connection_label.pack(anchor="e")
        self.header_random_label = ttk.Label(
            header_status,
            textvariable=self.random_status_var,
            foreground=self.ORANGE,
            background=self.SURFACE,
            font=("Segoe UI Semibold", 9),
        )
        self.header_random_label.pack(anchor="e", pady=(4, 0))

        connection = ttk.Frame(self, style="Card.TFrame", padding=(18, 13))
        connection.grid(row=1, column=0, sticky="ew", padx=18, pady=(14, 10))
        connection.grid_columnconfigure(6, weight=1)
        ttk.Label(connection, text="Port", style="Card.TLabel").grid(row=0, column=0, padx=(0, 7))
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=22)
        self.port_combo.grid(row=0, column=1, padx=(0, 7))
        ttk.Button(connection, text="Obnovit", style="Secondary.TButton", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 12))
        self.connect_button = ttk.Button(connection, text="Připojit", style="Accent.TButton", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=(0, 18))
        ttk.Label(connection, text="Profil", style="Card.TLabel").grid(row=0, column=4, padx=(0, 7))
        profile = ttk.Combobox(connection, textvariable=self.profile_var, values=(PROFILE_CURRENT, PROFILE_FIXED), state="readonly", width=36)
        profile.grid(row=0, column=5, padx=(0, 15))
        profile.bind("<<ComboboxSelected>>", lambda _event: self.update_preview())
        ttk.Button(connection, text="PING / HELLO", style="Secondary.TButton", command=self.send_ping).grid(row=0, column=7, padx=(0, 8))
        ttk.Button(connection, text="STOP CÍVKY", style="Danger.TButton", command=self.send_emergency_stop).grid(row=0, column=8)

        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=2, column=0, sticky="nsew", padx=18)

        commands_tab = ttk.Frame(self.notebook, style="Surface.TFrame")
        commands_tab.grid_rowconfigure(0, weight=1)
        commands_tab.grid_columnconfigure(0, weight=1)
        random_tab = ttk.Frame(self.notebook, style="Surface.TFrame")
        self.notebook.add(commands_tab, text="  UART PŘÍKAZY  ")
        self.notebook.add(random_tab, text="  RANDOM MOTION  ")

        body = ttk.Frame(commands_tab, style="Surface.TFrame")
        body.grid(row=0, column=0, sticky="nsew")
        body.grid_rowconfigure(0, weight=1)
        body.grid_columnconfigure(0, weight=0)
        body.grid_columnconfigure(1, weight=3)
        body.grid_columnconfigure(2, weight=2)

        left = ttk.Frame(body, style="Card.TFrame", padding=(14, 14))
        left.grid(row=0, column=0, sticky="ns", padx=(0, 10))
        ttk.Label(left, text="UART příkazy 0x00–0x16", style="CardTitle.TLabel").pack(anchor="w", pady=(0, 8))
        category = ttk.Combobox(
            left,
            textvariable=self.category_var,
            values=("Vše", "Cívky", "Motory", "Periferie", "Senzory", "Komunikace"),
            state="readonly",
            width=29,
        )
        category.pack(fill="x", pady=(0, 10))
        category.bind("<<ComboboxSelected>>", self._populate_command_list)
        self.command_list = tk.Listbox(
            left,
            width=34,
            height=18,
            bg="#FAF9FF",
            fg=self.TEXT,
            selectbackground=self.ACCENT_DARK,
            selectforeground="#FFFFFF",
            borderwidth=0,
            highlightthickness=1,
            highlightbackground=self.BORDER,
            activestyle="none",
            font=("Segoe UI", 10),
        )
        self.command_list.pack(fill="both", expand=True)
        self.command_list.bind("<<ListboxSelect>>", self._command_selected)
        self._populate_command_list()

        self.form_card = ttk.Frame(body, style="Card.TFrame", padding=(18, 16))
        self.form_card.grid(row=0, column=1, sticky="nsew", padx=(0, 10))
        self.form_card.grid_columnconfigure(1, weight=1)
        self.command_title = ttk.Label(self.form_card, text="", style="CardTitle.TLabel")
        self.command_title.grid(row=0, column=0, columnspan=3, sticky="w")
        self.command_summary = ttk.Label(self.form_card, text="", style="Hint.TLabel", wraplength=440)
        self.command_summary.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4, 12))
        self.command_caution = ttk.Label(self.form_card, text="", style="Warn.TLabel", wraplength=520)
        self.command_caution.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self.fields_container = ttk.Frame(self.form_card, style="Card.TFrame")
        self.fields_container.grid(row=3, column=0, columnspan=3, sticky="nsew")
        self.fields_container.grid_columnconfigure(1, weight=1)
        self.fields_container.grid_columnconfigure(4, weight=1)
        self.form_card.grid_rowconfigure(3, weight=1)

        send_row = ttk.Frame(self.form_card, style="Card.TFrame")
        send_row.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(14, 0))
        ttk.Label(send_row, text="SEQ", style="Card.TLabel").pack(side="left")
        sequence_entry = ttk.Entry(send_row, textvariable=self.sequence_var, width=6)
        sequence_entry.pack(side="left", padx=(7, 12))
        sequence_entry.bind("<KeyRelease>", lambda _event: self.update_preview())
        ttk.Checkbutton(send_row, text="automaticky +1", variable=self.auto_sequence_var).pack(side="left")
        ttk.Button(send_row, text="Odeslat příkaz", style="Accent.TButton", command=self.send_current_command).pack(side="right")

        right = ttk.Frame(body, style="Card.TFrame", padding=(18, 16))
        right.grid(row=0, column=2, sticky="nsew")
        right.grid_rowconfigure(2, weight=1)
        right.grid_columnconfigure(0, weight=1)
        ttk.Label(right, text="Náhled rámce", style="CardTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            right,
            text="CRC příkazu: 00 00 00 00 • escaping: 7D + nezměněný speciální byte",
            style="Warn.TLabel",
            wraplength=440,
        ).grid(row=1, column=0, sticky="ew", pady=(4, 10))
        self.preview = tk.Text(
            right,
            bg="#F8F5FF",
            fg="#4C1D95",
            insertbackground=self.TEXT,
            borderwidth=0,
            highlightthickness=1,
            highlightbackground=self.BORDER,
            font=("Cascadia Mono", 10),
            wrap="word",
            padx=12,
            pady=10,
            state="disabled",
        )
        self.preview.grid(row=2, column=0, sticky="nsew")

        self._build_random_motion_tab(random_tab)

        log_card = ttk.Frame(self, style="Card.TFrame", padding=(16, 12))
        log_card.grid(row=3, column=0, sticky="nsew", padx=18, pady=(10, 18))
        log_card.grid_rowconfigure(1, weight=1)
        log_card.grid_columnconfigure(0, weight=1)
        log_header = ttk.Frame(log_card, style="Card.TFrame")
        log_header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(log_header, text="Komunikační log", style="CardTitle.TLabel").pack(side="left")
        ttk.Button(log_header, text="Vymazat", style="Secondary.TButton", command=self.clear_log).pack(side="right")
        self.log = tk.Text(
            log_card,
            height=8,
            bg="#FCFBFF",
            fg=self.TEXT,
            insertbackground=self.TEXT,
            borderwidth=0,
            highlightthickness=1,
            highlightbackground=self.BORDER,
            font=("Cascadia Mono", 9),
            wrap="word",
            padx=10,
            pady=8,
            state="disabled",
        )
        self.log.grid(row=1, column=0, sticky="nsew")
        self.log.tag_configure("TX", foreground=self.ACCENT)
        self.log.tag_configure("RX", foreground=self.GREEN)
        self.log.tag_configure("ERR", foreground=self.RED)
        self.log.tag_configure("INFO", foreground=self.MUTED)
        self.log.tag_configure("SIM", foreground=self.ORANGE)
        raw_row = ttk.Frame(log_card, style="Card.TFrame")
        raw_row.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        raw_row.grid_columnconfigure(1, weight=1)
        ttk.Label(raw_row, text="RAW HEX", style="Card.TLabel").grid(row=0, column=0, padx=(0, 8))
        self.raw_hex_var = tk.StringVar()
        raw_entry = ttk.Entry(raw_row, textvariable=self.raw_hex_var)
        raw_entry.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        raw_entry.bind("<Return>", lambda _event: self.send_raw_hex())
        ttk.Button(raw_row, text="Odeslat přesně", style="Secondary.TButton", command=self.send_raw_hex).grid(row=0, column=2)
        self._log("INFO", "GUI připraveno. Současný firmware má vypnutou kontrolu vstupního CRC.")

    def _build_random_motion_tab(self, tab: ttk.Frame) -> None:
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=3)
        tab.grid_columnconfigure(1, weight=2)

        settings = ttk.Frame(tab, style="Card.TFrame", padding=(22, 18))
        settings.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=(10, 0))
        for column in (1, 3, 5):
            settings.grid_columnconfigure(column, weight=1)

        ttk.Label(settings, text="Random Motion", style="CardTitle.TLabel", font=("Segoe UI Semibold", 16)).grid(
            row=0, column=0, columnspan=6, sticky="w"
        )
        ttk.Label(
            settings,
            text="Automatická změna azimutu a elevace pomocí rotačního pole (CMD 0x06).",
            style="Hint.TLabel",
        ).grid(row=1, column=0, columnspan=6, sticky="w", pady=(3, 14))

        def section(row: int, title: str) -> None:
            ttk.Label(settings, text=title, style="Section.TLabel").grid(
                row=row, column=0, columnspan=6, sticky="w", pady=(10, 5)
            )

        def field(row: int, pair: int, label: str, variable: tk.StringVar, unit: str = "") -> ttk.Entry:
            base = pair * 2
            ttk.Label(settings, text=label, style="Card.TLabel").grid(row=row, column=base, sticky="w", padx=(0, 7), pady=4)
            holder = ttk.Frame(settings, style="Card.TFrame")
            holder.grid(row=row, column=base + 1, sticky="ew", padx=(0, 14), pady=4)
            holder.grid_columnconfigure(0, weight=1)
            entry = ttk.Entry(holder, textvariable=variable, width=10)
            entry.grid(row=0, column=0, sticky="ew")
            if unit:
                ttk.Label(holder, text=unit, style="Hint.TLabel").grid(row=0, column=1, padx=(5, 0))
            return entry

        section(2, "Průběh")
        ttk.Label(settings, text="Režim", style="Card.TLabel").grid(row=3, column=0, sticky="w", padx=(0, 7), pady=4)
        mode = ttk.Combobox(
            settings,
            textvariable=self.random_mode_var,
            values=("Úplně náhodně", "Plynulé kroky"),
            state="readonly",
            width=17,
        )
        mode.grid(row=3, column=1, sticky="ew", padx=(0, 14), pady=4)
        mode.bind("<<ComboboxSelected>>", self._random_mode_changed)
        field(3, 1, "Trvání", self.random_duration_var, "s")
        field(3, 2, "Změna každých", self.random_interval_var, "s")

        section(4, "Rozsah pohybu")
        field(5, 0, "Azimut MIN", self.random_az_min_var, "°")
        field(5, 1, "Azimut MAX", self.random_az_max_var, "°")
        self.random_step_az_entry = field(5, 2, "Krok azimutu", self.random_step_az_var, "°")
        field(6, 0, "Elevace MIN", self.random_el_min_var, "°")
        field(6, 1, "Elevace MAX", self.random_el_max_var, "°")
        self.random_step_el_entry = field(6, 2, "Krok elevace", self.random_step_el_var, "°")

        section(7, "Magnetické pole")
        field(8, 0, "Amplituda", self.random_amplitude_var, "µT")
        field(8, 1, "Frekvence", self.random_frequency_var, "Hz")
        field(8, 2, "Rampa", self.random_ramp_var, "×0,1 s")

        options = ttk.Frame(settings, style="Card.TFrame")
        options.grid(row=9, column=0, columnspan=6, sticky="ew", pady=(14, 4))
        ttk.Checkbutton(
            options,
            text="Simulace – neposílat na UART",
            variable=self.random_simulation_var,
        ).pack(side="left", padx=(0, 18))
        ttk.Checkbutton(
            options,
            text="Při startu zapnout všechny cívky",
            variable=self.random_enable_outputs_var,
        ).pack(side="left", padx=(0, 18))
        ttk.Checkbutton(
            options,
            text="Po dokončení cívky vypnout",
            variable=self.random_power_off_finish_var,
        ).pack(side="left")

        ttk.Label(
            settings,
            text="Bezpečný začátek: simulace je zapnutá. Pro reálný pohyb ji vědomě vypněte.",
            style="Warn.TLabel",
        ).grid(row=10, column=0, columnspan=6, sticky="w", pady=(5, 12))

        controls = ttk.Frame(settings, style="Card.TFrame")
        controls.grid(row=11, column=0, columnspan=6, sticky="ew")
        self.random_start_button = ttk.Button(
            controls,
            text="▶  SPUSTIT RANDOM MOTION",
            style="Accent.TButton",
            command=self.start_random_motion,
        )
        self.random_start_button.pack(side="left")
        self.random_pause_button = ttk.Button(
            controls,
            text="■  ZASTAVIT ZMĚNY",
            style="Secondary.TButton",
            command=self.stop_random_motion,
            state="disabled",
        )
        self.random_pause_button.pack(side="left", padx=(10, 0))

        monitor = ttk.Frame(tab, style="Card.TFrame", padding=(22, 18))
        monitor.grid(row=0, column=1, sticky="nsew", pady=(10, 0))
        monitor.grid_columnconfigure(0, weight=1)
        monitor.grid_columnconfigure(1, weight=1)
        monitor.grid_rowconfigure(3, weight=1)

        ttk.Label(monitor, text="Živý náhled pole", style="CardTitle.TLabel", font=("Segoe UI Semibold", 16)).grid(
            row=0, column=0, sticky="w"
        )
        self.random_status_label = ttk.Label(
            monitor,
            textvariable=self.random_status_var,
            background="#FEF3C7",
            foreground=self.ORANGE,
            font=("Segoe UI Semibold", 10),
            padding=(10, 5),
        )
        self.random_status_label.grid(row=0, column=1, sticky="e")

        metrics = ttk.Frame(monitor, style="Card.TFrame")
        metrics.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(16, 8))
        for column in range(4):
            metrics.grid_columnconfigure(column, weight=1)
        for column, (title, variable) in enumerate((
            ("AZIMUT", self.random_current_az_var),
            ("ELEVACE", self.random_current_el_var),
            ("ZBÝVÁ", self.random_remaining_var),
            ("DALŠÍ ZMĚNA", self.random_next_var),
        )):
            ttk.Label(metrics, text=title, style="Hint.TLabel").grid(row=0, column=column, sticky="w")
            ttk.Label(metrics, textvariable=variable, style="Metric.TLabel").grid(row=1, column=column, sticky="w", padx=(0, 10))

        self.random_canvas = tk.Canvas(
            monitor,
            width=420,
            height=260,
            bg="#FAF9FF",
            highlightthickness=1,
            highlightbackground=self.BORDER,
        )
        self.random_canvas.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(8, 12))
        self.random_canvas.bind("<Configure>", lambda _event: self._draw_random_field())

        ttk.Progressbar(
            monitor,
            variable=self.random_progress_var,
            maximum=100,
            style="Purple.Horizontal.TProgressbar",
        ).grid(row=4, column=0, columnspan=2, sticky="ew", pady=(0, 14))

        ttk.Button(
            monitor,
            text="■  STOP RANDOM + CÍVKY OFF",
            style="Danger.TButton",
            command=self.random_emergency_stop,
        ).grid(row=5, column=0, columnspan=2, sticky="ew")
        ttk.Label(
            monitor,
            text="Zastavit změny ponechá poslední pole aktivní. Červený STOP navíc odešle 0x07 / ALL / STOP.",
            style="Hint.TLabel",
            wraplength=430,
        ).grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        self._random_mode_changed()
        self.after_idle(self._draw_random_field)

    def _random_mode_changed(self, _event: Any = None) -> None:
        state = "disabled" if self.random_mode_var.get() == "Úplně náhodně" else "normal"
        if hasattr(self, "random_step_az_entry"):
            self.random_step_az_entry.configure(state=state)
            self.random_step_el_entry.configure(state=state)

    @staticmethod
    def _random_number(variable: tk.StringVar, label: str) -> float:
        try:
            return float(variable.get().strip().replace(",", "."))
        except ValueError as exc:
            raise ValueError(f"{label}: zadejte platné číslo.") from exc

    def _random_config_from_form(self) -> dict[str, float | str | bool]:
        config: dict[str, float | str | bool] = {
            "mode": self.random_mode_var.get(),
            "duration": self._random_number(self.random_duration_var, "Trvání"),
            "interval": self._random_number(self.random_interval_var, "Interval"),
            "az_min": self._random_number(self.random_az_min_var, "Azimut MIN"),
            "az_max": self._random_number(self.random_az_max_var, "Azimut MAX"),
            "el_min": self._random_number(self.random_el_min_var, "Elevace MIN"),
            "el_max": self._random_number(self.random_el_max_var, "Elevace MAX"),
            "step_az": self._random_number(self.random_step_az_var, "Krok azimutu"),
            "step_el": self._random_number(self.random_step_el_var, "Krok elevace"),
            "amplitude": self._random_number(self.random_amplitude_var, "Amplituda"),
            "frequency": self._random_number(self.random_frequency_var, "Frekvence"),
            "ramp": self._random_number(self.random_ramp_var, "Rampa"),
            "simulation": self.random_simulation_var.get(),
            "enable_outputs": self.random_enable_outputs_var.get(),
            "power_off_finish": self.random_power_off_finish_var.get(),
        }
        if float(config["duration"]) <= 0:
            raise ValueError("Trvání musí být větší než 0 s.")
        if float(config["interval"]) <= 0:
            raise ValueError("Interval změny musí být větší než 0 s.")
        # GUI se obnovuje po 50 ms; 100 ms je bezpečné minimum pro UART i log.
        config["interval"] = max(0.1, float(config["interval"]))
        if not 0 <= float(config["az_min"]) <= float(config["az_max"]) <= 360:
            raise ValueError("Azimut musí splňovat 0 ≤ MIN ≤ MAX ≤ 360°.")
        if not -180 <= float(config["el_min"]) <= float(config["el_max"]) <= 180:
            raise ValueError("Elevace musí splňovat −180 ≤ MIN ≤ MAX ≤ 180°.")
        if float(config["step_az"]) < 0 or float(config["step_el"]) < 0:
            raise ValueError("Velikost plynulého kroku nesmí být záporná.")
        if not 0 <= float(config["amplitude"]) <= 12000:
            raise ValueError("Amplituda musí být v rozsahu 0…12000 µT.")
        if not 0 <= float(config["frequency"]) <= 100:
            raise ValueError("Frekvence musí být v rozsahu 0…100 Hz.")
        if not 0 <= float(config["ramp"]) <= 200:
            raise ValueError("Rampa musí být v rozsahu 0…200 × 0,1 s.")
        return config

    def start_random_motion(self) -> None:
        try:
            config = self._random_config_from_form()
        except ValueError as exc:
            messagebox.showerror("Random Motion – neplatné nastavení", str(exc))
            return
        if not bool(config["simulation"]) and self.serial_port is None:
            messagebox.showwarning(
                "UART není připojen",
                "Pro reálný Random Motion nejdříve připojte COM port, nebo zapněte simulační režim.",
            )
            return
        if (
            not bool(config["simulation"])
            and bool(config["enable_outputs"])
            and not messagebox.askyesno(
                "Spustit reálné magnetické pole",
                f"Random Motion zapne všechny výstupy cívek.\n\n"
                f"Amplituda: {float(config['amplitude']):g} µT\n"
                f"Frekvence: {float(config['frequency']):g} Hz\n"
                f"Trvání: {float(config['duration']):g} s\n\n"
                "Je pracovní prostor připravený?",
            )
        ):
            return

        self._halt_random_motion("PŘIPRAVENO", write_log=False)
        self.random_config = config
        self.random_running = True
        now = time.monotonic()
        self.random_started_at = now
        self.random_ends_at = now + float(config["duration"])
        self.random_next_change_at = now
        self.random_current_az_value = (float(config["az_min"]) + float(config["az_max"])) / 2.0
        self.random_current_el_value = (float(config["el_min"]) + float(config["el_max"])) / 2.0
        self.random_progress_var.set(0.0)
        self._set_random_status("● BĚŽÍ", self.GREEN, "#D1FAE5")
        self.random_start_button.configure(state="disabled")
        self.random_pause_button.configure(state="normal")
        mode_text = "SIMULACE" if bool(config["simulation"]) else "UART"
        self._log(
            "INFO",
            f"RANDOM MOTION START · {mode_text} · {config['mode']} · "
            f"trvání {float(config['duration']):g} s · interval {float(config['interval']):g} s",
        )

        # Původní Random Motion nejprve zvolí jednotky magnetického pole.
        if not self._send_random_command(0x00, bytes((2,)), "RANDOM jednotky vstupu = µT"):
            self._halt_random_motion("CHYBA", write_log=True)
            return
        if not self._random_motion_step():
            self._halt_random_motion("CHYBA", write_log=True)
            return
        if bool(config["enable_outputs"]):
            if not self._send_random_command(0x07, bytes((3, 1)), "RANDOM cívky ALL = RUN"):
                self._halt_random_motion("CHYBA", write_log=True)
                return
        self.random_next_change_at = now + float(config["interval"])
        self._schedule_random_loop()

    def _send_random_command(self, command_id: int, payload: bytes, description: str) -> bool:
        try:
            sequence = int(self.sequence_var.get().strip(), 0)
            _raw, wire = build_command(command_id, sequence, payload, self.profile_var.get())
        except Exception as exc:
            self._log("ERR", f"Random Motion nelze sestavit: {exc}")
            return False

        if bool(self.random_config.get("simulation", self.random_simulation_var.get())):
            self._log("SIM", f"SIM {description}, SEQ {sequence}: {hex_bytes(wire)}")
            success = True
        else:
            success = self._write_frame(wire, f"{description}, SEQ {sequence}")
        if success:
            self.sequence_var.set(str((sequence + 1) & 0xFF))
        return success

    def _random_motion_step(self) -> bool:
        config = self.random_config
        if config.get("mode") == "Úplně náhodně":
            azimuth = random.uniform(float(config["az_min"]), float(config["az_max"]))
            elevation = random.uniform(float(config["el_min"]), float(config["el_max"]))
        else:
            azimuth = self.random_current_az_value + random.uniform(-float(config["step_az"]), float(config["step_az"]))
            elevation = self.random_current_el_value + random.uniform(-float(config["step_el"]), float(config["step_el"]))
            azimuth = min(float(config["az_max"]), max(float(config["az_min"]), azimuth))
            elevation = min(float(config["el_max"]), max(float(config["el_min"]), elevation))

        self.random_current_az_value = float(round(azimuth))
        self.random_current_el_value = float(round(elevation))
        self.random_current_az_var.set(f"{self.random_current_az_value:.0f}°")
        self.random_current_el_var.set(f"{self.random_current_el_value:+.0f}°")
        self._draw_random_field()

        values = {
            "azimuth": int(self.random_current_az_value),
            "elevation": int(self.random_current_el_value),
            "amplitude": int(round(float(config["amplitude"]))),
            "frequency": int(round(float(config["frequency"]))),
            "offset_x": 0,
            "offset_y": 0,
            "offset_z": 0,
            "ramp": int(round(float(config["ramp"]))),
        }
        payload = encode_payload(COMMAND_BY_ID[0x06], values)
        return self._send_random_command(
            0x06,
            payload,
            f"RANDOM azimut {values['azimuth']}° · elevace {values['elevation']}°",
        )

    def _schedule_random_loop(self) -> None:
        if self.random_after_id is not None:
            self.after_cancel(self.random_after_id)
        self.random_after_id = self.after(50, self._random_motion_loop)

    def _random_motion_loop(self) -> None:
        self.random_after_id = None
        if not self.random_running:
            return
        now = time.monotonic()
        remaining = max(0.0, self.random_ends_at - now)
        duration = float(self.random_config["duration"])
        self.random_remaining_var.set(self._format_random_time(remaining))
        self.random_progress_var.set(min(100.0, max(0.0, 100.0 * (1.0 - remaining / duration))))

        if remaining <= 0:
            power_off = bool(self.random_config.get("power_off_finish", True))
            self._halt_random_motion("DOKONČENO", write_log=False)
            self.random_progress_var.set(100.0)
            self.random_remaining_var.set("00:00.0")
            self.random_next_var.set("—")
            if power_off:
                self._send_random_command(0x07, bytes((3, 0)), "RANDOM dokončeno · cívky ALL = STOP")
            self._log("INFO", "RANDOM MOTION STOP · nastavené trvání dokončeno.")
            return

        if now >= self.random_next_change_at:
            if not self._random_motion_step():
                self._halt_random_motion("CHYBA", write_log=True)
                return
            self.random_next_change_at = now + float(self.random_config["interval"])
        self.random_next_var.set(f"{max(0.0, self.random_next_change_at - now):.1f} s")
        self._schedule_random_loop()

    @staticmethod
    def _format_random_time(seconds: float) -> str:
        tenths = max(0, int(math.ceil(seconds * 10.0)))
        minutes = tenths // 600
        remainder = (tenths % 600) / 10.0
        return f"{minutes:02d}:{remainder:04.1f}"

    def _set_random_status(self, text: str, foreground: str, background: str) -> None:
        self.random_status_var.set(text)
        if hasattr(self, "random_status_label"):
            self.random_status_label.configure(foreground=foreground, background=background)
        if hasattr(self, "header_random_label"):
            self.header_random_label.configure(foreground=foreground)

    def _halt_random_motion(self, status: str, write_log: bool) -> bool:
        was_running = self.random_running
        self.random_running = False
        if self.random_after_id is not None:
            try:
                self.after_cancel(self.random_after_id)
            except tk.TclError:
                pass
            self.random_after_id = None
        if hasattr(self, "random_start_button"):
            self.random_start_button.configure(state="normal")
            self.random_pause_button.configure(state="disabled")
        color = self.RED if status in ("CHYBA", "NOUZOVÝ STOP") else self.ORANGE
        background = "#FEE2E2" if color == self.RED else "#FEF3C7"
        self._set_random_status(f"● {status}", color, background)
        if was_running and write_log:
            self._log("INFO", f"RANDOM MOTION STOP · {status.lower()}.")
        return was_running

    def stop_random_motion(self) -> None:
        self._halt_random_motion("ZASTAVENO", write_log=True)
        self.random_next_var.set("—")

    def random_emergency_stop(self) -> None:
        self._halt_random_motion("NOUZOVÝ STOP", write_log=True)
        self.random_next_var.set("—")
        # Bezpečnostní STOP jde na UART vždy, pokud je port připojený – i když
        # byl samotný Random Motion spuštěný pouze v simulaci.
        self.random_config["simulation"] = self.serial_port is None
        self._send_random_command(0x07, bytes((3, 0)), "STOP RANDOM + cívky ALL = STOP")

    def _draw_random_field(self) -> None:
        if not hasattr(self, "random_canvas"):
            return
        canvas = self.random_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 360)
        height = max(canvas.winfo_height(), 230)
        center_x = width * 0.43
        center_y = height * 0.52
        radius = max(65.0, min(width * 0.31, height * 0.38))

        canvas.create_oval(
            center_x - radius,
            center_y - radius,
            center_x + radius,
            center_y + radius,
            fill="#F3E8FF",
            outline=self.BORDER,
            width=2,
        )
        canvas.create_line(center_x - radius, center_y, center_x + radius, center_y, fill="#C4B5FD")
        canvas.create_line(center_x, center_y - radius, center_x, center_y + radius, fill="#C4B5FD")
        for label, x, y in (
            ("0°", center_x, center_y - radius - 12),
            ("90°", center_x + radius + 20, center_y),
            ("180°", center_x, center_y + radius + 13),
            ("270°", center_x - radius - 24, center_y),
        ):
            canvas.create_text(x, y, text=label, fill=self.MUTED, font=("Segoe UI", 8))

        planar = max(0.12, abs(math.cos(math.radians(self.random_current_el_value))))
        angle = math.radians(self.random_current_az_value - 90.0)
        vector_length = radius * 0.88 * planar
        end_x = center_x + vector_length * math.cos(angle)
        end_y = center_y + vector_length * math.sin(angle)
        canvas.create_line(
            center_x,
            center_y,
            end_x,
            end_y,
            fill=self.ACCENT,
            width=5,
            arrow=tk.LAST,
            arrowshape=(12, 14, 6),
        )
        canvas.create_oval(center_x - 5, center_y - 5, center_x + 5, center_y + 5, fill=self.ACCENT_DARK, outline="")

        gauge_x = width - 48
        gauge_top = center_y - radius
        gauge_bottom = center_y + radius
        canvas.create_line(gauge_x, gauge_top, gauge_x, gauge_bottom, fill="#C4B5FD", width=6)
        normalized_el = (max(-180.0, min(180.0, self.random_current_el_value)) + 180.0) / 360.0
        marker_y = gauge_bottom - normalized_el * (gauge_bottom - gauge_top)
        canvas.create_oval(gauge_x - 8, marker_y - 8, gauge_x + 8, marker_y + 8, fill=self.ORANGE, outline="#FFFFFF", width=2)
        canvas.create_text(gauge_x, gauge_top - 14, text="+180°", fill=self.MUTED, font=("Segoe UI", 8))
        canvas.create_text(gauge_x, gauge_bottom + 14, text="−180°", fill=self.MUTED, font=("Segoe UI", 8))
        canvas.create_text(gauge_x, center_y, text="0°", fill=self.MUTED, font=("Segoe UI", 8), anchor="w")

    def _populate_command_list(self, _event: Any = None) -> None:
        selected_id = self.current_command.command_id
        category = self.category_var.get()
        self.visible_command_indices = [
            index for index, command in enumerate(COMMANDS)
            if category == "Vše" or command.category == category
        ]
        self.command_list.delete(0, "end")
        selected_row = 0
        for row, command_index in enumerate(self.visible_command_indices):
            command = COMMANDS[command_index]
            self.command_list.insert("end", f"  0x{command.command_id:02X}  {command.title}")
            if command.command_id == selected_id:
                selected_row = row
        if self.visible_command_indices:
            self.command_list.selection_set(selected_row)
            self.command_list.activate(selected_row)
            # Při prvním sestavování okna ještě pravá část formuláře neexistuje.
            # __init__ ji po dokončení _build_ui() zobrazí pomocí show_command(0).
            if hasattr(self, "command_title"):
                self.show_command(self.visible_command_indices[selected_row])

    def _command_selected(self, _event: Any = None) -> None:
        selection = self.command_list.curselection()
        if selection:
            self.show_command(self.visible_command_indices[int(selection[0])])

    def show_command(self, index: int) -> None:
        self.current_command = COMMANDS[index]
        self.command_title.configure(
            text=f"{self.current_command.category}  ·  0x{self.current_command.command_id:02X}  {self.current_command.title}"
        )
        self.command_summary.configure(text=self.current_command.summary)
        caution = self.current_command.caution
        if self.current_command.command_id in CURRENT_FIRMWARE_OVERSIZE:
            caution = (caution + " " if caution else "") + "Aktuální 32B RX buffer tento příkaz nepřijme; použijte opravený firmware."
        if self.current_command.command_id in CURRENT_FIRMWARE_LENGTH_CONFLICT:
            caution = (caution + " " if caution else "") + "Aktuální tabulka vyžaduje délku 0 B, ale obsluha potřebuje 1 B; příkaz vyžaduje opravu firmware."
        self.command_caution.configure(text=f"⚠ {caution}" if caution else "")
        for child in self.fields_container.winfo_children():
            child.destroy()
        self.field_vars.clear()

        field_count = len(self.current_command.fields)
        rows_per_column = (field_count + 1) // 2 if field_count > 8 else field_count
        for index, field in enumerate(self.current_command.fields):
            column_group = index // rows_per_column if field_count > 8 else 0
            row = index % rows_per_column if field_count > 8 else index
            base_column = column_group * 3
            label_pad = (18, 12) if column_group else (0, 12)
            ttk.Label(self.fields_container, text=field.label, style="Card.TLabel").grid(
                row=row, column=base_column, sticky="w", padx=label_pad, pady=4
            )
            variable = tk.StringVar()
            self.field_vars[field.key] = variable
            if field.choices:
                labels = [label for label, _value in field.choices]
                default_label = next(label for label, value in field.choices if value == field.default)
                variable.set(default_label)
                widget = ttk.Combobox(self.fields_container, textvariable=variable, values=labels, state="readonly", width=22)
                widget.bind("<<ComboboxSelected>>", lambda _event: self.update_preview())
            else:
                variable.set(str(field.default))
                widget = ttk.Entry(self.fields_container, textvariable=variable, width=24)
                widget.bind("<KeyRelease>", lambda _event: self.update_preview())
            widget.grid(row=row, column=base_column + 1, sticky="ew", pady=4)
            note = field.unit
            if field.hint:
                note = f"{note}  {field.hint}".strip()
            ttk.Label(self.fields_container, text=note, style="Hint.TLabel", wraplength=125).grid(
                row=row, column=base_column + 2, sticky="w", padx=(8, 0), pady=4
            )
        self.update_preview()

    def _values_from_form(self) -> dict[str, Any]:
        values: dict[str, Any] = {}
        for field in self.current_command.fields:
            value: Any = self.field_vars[field.key].get()
            if field.choices:
                choice_map = dict(field.choices)
                if value not in choice_map:
                    raise ValueError(f"{field.label}: neplatná volba.")
                value = choice_map[value]
            values[field.key] = value
        return values

    def _current_frame(self) -> tuple[bytes, bytes, bytes]:
        sequence = int(self.sequence_var.get().strip(), 0)
        payload = encode_payload(self.current_command, self._values_from_form())
        raw, wire = build_command(self.current_command.command_id, sequence, payload, self.profile_var.get())
        return payload, raw, wire

    def update_preview(self) -> None:
        try:
            payload, raw, wire = self._current_frame()
            wire_payload_length = int.from_bytes(raw[2:4], "big")
            compatibility = ""
            if wire_payload_length != len(payload):
                compatibility = f" → na UART {wire_payload_length} B (kompatibilní doplnění/úprava)"
            buffer_warning = ""
            if self.profile_var.get() == PROFILE_CURRENT and self.current_command.command_id in CURRENT_FIRMWARE_OVERSIZE:
                buffer_warning = "\n⚠ BLOKOVÁNO: rámec se nevejde do 32B RX bufferu aktuálního firmware.\n"
            if self.profile_var.get() == PROFILE_CURRENT and self.current_command.command_id in CURRENT_FIRMWARE_LENGTH_CONFLICT:
                buffer_warning = "\n⚠ BLOKOVÁNO: tabulka délky 0 B je v konfliktu s 1B payloadem obsluhy.\n"
            text = (
                f"CMD  0x{self.current_command.command_id:02X}\n"
                f"SEQ  {int(self.sequence_var.get().strip(), 0)}\n"
                f"PLD  {len(payload)} B logicky{compatibility}\n"
                f"LEN  {hex_bytes(raw[2:4])} (big-endian)\n"
                f"PROFIL  {self.profile_var.get()}\n"
                f"{buffer_warning}\n"
                f"LOGICKÝ PAYLOAD\n{hex_bytes(payload) or '—'}\n\n"
                f"RAW BODY\n{hex_bytes(raw)}\n\n"
                f"UART FRAME ({len(wire)} B)\n{hex_bytes(wire)}"
            )
        except Exception as exc:
            text = f"Rámec zatím nelze sestavit:\n{exc}"
        self.preview.configure(state="normal")
        self.preview.delete("1.0", "end")
        self.preview.insert("1.0", text)
        self.preview.configure(state="disabled")

    def refresh_ports(self) -> None:
        if list_ports is None:
            self.port_combo.configure(values=())
            return
        ports = [port.device for port in list_ports.comports()]
        self.port_combo.configure(values=ports)
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        if self.serial_port is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        if serial is None:
            messagebox.showerror("Chybí pyserial", "Nainstalujte závislost:\n\npython -m pip install pyserial")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Není vybrán port", "Vyberte nebo napište název sériového portu.")
            return
        opened_port: Any = None
        try:
            # Port nejprve nakonfigurujeme v zavřeném stavu. Konstruktor
            # Serial(port, ...) by jej otevřel s výchozím DTR/RTS a jejich
            # následné vypnutí může přes některé převodníky resetovat STM32.
            opened_port = serial.Serial()
            opened_port.port = port
            opened_port.baudrate = BAUDRATE
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
            messagebox.showerror("Připojení selhalo", str(exc))
            return
        self.frame_parser = FrameStreamParser()
        self.ascii_rx_buffer.clear()
        self.pending_tx.clear()
        self.waiting_for_ready = True
        self.startup_wait_deadline = time.monotonic() + 1.5
        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.connection_var.set(f"●  Připojeno: {port}")
        self.connection_label.configure(foreground=self.GREEN)
        self.connect_button.configure(text="Odpojit", style="Secondary.TButton")
        self._log("INFO", f"Připojeno k {port} @ {BAUDRATE} Bd.")

    def disconnect(self) -> None:
        if self.random_running:
            self._halt_random_motion("ODPOJENO", write_log=True)
            self.random_next_var.set("—")
        self.reader_stop.set()
        port = self.serial_port
        self.serial_port = None
        self.waiting_for_ready = False
        self.pending_tx.clear()
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        self.connection_var.set("●  Odpojeno")
        self.connection_label.configure(foreground=self.RED)
        self.connect_button.configure(text="Připojit", style="Accent.TButton")
        self._log("INFO", "Sériový port odpojen.")

    def _reader_loop(self) -> None:
        while not self.reader_stop.is_set():
            port = self.serial_port
            if port is None:
                break
            try:
                waiting = port.in_waiting
                data = port.read(waiting or 1)
                if data:
                    self.rx_queue.put(("data", bytes(data)))
            except Exception as exc:
                self.rx_queue.put(("error", exc))
                break

    def _poll_serial(self) -> None:
        try:
            while True:
                kind, value = self.rx_queue.get_nowait()
                if kind == "error":
                    self._log("ERR", f"Chyba UART: {value}")
                    self.disconnect()
                else:
                    data = value
                    self._log("RX", f"RX bytes ({len(data)} B): {hex_bytes(data)}")
                    frames = self.frame_parser.feed(data)
                    outside = self.frame_parser.drain_outside()
                    for frame in frames:
                        try:
                            if len(frame) >= 2 and frame[0] == 0xFF and 0x1E <= frame[1] <= 0x2D:
                                self.waiting_for_ready = True
                                self.startup_wait_deadline = time.monotonic() + 2.0
                            self._log("RX", "RX frame:\n" + decode_response(frame))
                        except Exception as exc:
                            # Jeden poškozený/neznámý rámec nesmí zastavit
                            # zpracování všech dalších UART dat.
                            self._log("ERR", f"Rámec nelze dekódovat: {exc}\nRAW: {hex_bytes(frame)}")
                    # Text zpracujeme až po rámcích. Pokud jsou poslední
                    # notifikace a "STM32 ready!!!" ve stejném čtení,
                    # ready tak správně ukončí čekání na self-test.
                    if outside:
                        self._handle_ascii_rx(outside)
        except queue.Empty:
            pass
        except Exception as exc:
            self._log("ERR", f"Chyba RX parseru: {exc}")
            self.frame_parser = FrameStreamParser()
        finally:
            if self.waiting_for_ready and time.monotonic() >= self.startup_wait_deadline:
                self._device_ready("timeout po self-testu")
            self.after(50, self._poll_serial)

    def _handle_ascii_rx(self, data: bytes) -> None:
        self.ascii_rx_buffer.extend(data)
        if len(self.ascii_rx_buffer) > 512:
            del self.ascii_rx_buffer[:-256]
        printable = "".join(
            chr(byte) if byte in b"\r\n\t" or 32 <= byte < 127 else f"\\x{byte:02X}"
            for byte in data
        ).strip()
        if printable:
            self._log("RX", f"RX text: {printable}")
        if b"STM32 ready!!!" in self.ascii_rx_buffer:
            self._device_ready("STM32 ready!!!")
            self.ascii_rx_buffer.clear()

    def _device_ready(self, reason: str) -> None:
        was_waiting = self.waiting_for_ready
        self.waiting_for_ready = False
        if self.serial_port is not None:
            self.connection_var.set(f"●  STM32 připraveno: {self.port_var.get().strip()}")
            self.connection_label.configure(foreground=self.GREEN)
        if was_waiting:
            self._log("INFO", f"Start zařízení dokončen ({reason}).")
        self._flush_pending_tx()

    def _flush_pending_tx(self) -> None:
        if not self.pending_tx or self.serial_port is None:
            return
        pending = self.pending_tx[:]
        self.pending_tx.clear()
        self._log("INFO", f"Odesílám {len(pending)} příkaz(ů) čekajících na dokončení self-testu.")
        for wire, description in pending:
            self._write_frame_now(wire, description)

    def _write_frame(self, wire: bytes, description: str) -> bool:
        if self.serial_port is None:
            messagebox.showwarning("UART není připojen", "Nejdříve se připojte k sériovému portu.")
            return False
        if self.waiting_for_ready:
            if len(self.pending_tx) >= 20:
                messagebox.showwarning("Fronta je plná", "Počkejte na dokončení self-testu STM32.")
                return False
            self.pending_tx.append((wire, description))
            self._log("INFO", f"TX čeká na dokončení self-testu: {description}")
            return True
        return self._write_frame_now(wire, description)

    def _write_frame_now(self, wire: bytes, description: str) -> bool:
        if self.serial_port is None:
            return False
        try:
            written = self.serial_port.write(wire)
            self.serial_port.flush()
            self._log("TX", f"TX {description} ({written} B):\n{hex_bytes(wire)}")
            return written == len(wire)
        except Exception as exc:
            self._log("ERR", f"Odeslání selhalo: {exc}")
            messagebox.showerror("Odeslání selhalo", str(exc))
            return False

    def send_current_command(self) -> None:
        try:
            values = self._values_from_form()
            _payload, _raw, wire = self._current_frame()
        except Exception as exc:
            messagebox.showerror("Neplatné parametry", str(exc))
            return
        command_id = self.current_command.command_id
        if self.profile_var.get() == PROFILE_CURRENT and command_id in CURRENT_FIRMWARE_OVERSIZE:
            messagebox.showerror(
                "Příkaz se nevejde do RX bufferu",
                "Aktuální firmware má 32B přijímací buffer a tento rámec je delší.\n\n"
                "Zvětšete COMM_MAX_FRAME_LEN alespoň na 80 B, opravte tabulku délek "
                "a potom v GUI zvolte profil opraveného firmware.",
            )
            return
        if self.profile_var.get() == PROFILE_CURRENT and command_id in CURRENT_FIRMWARE_LENGTH_CONFLICT:
            messagebox.showerror(
                "Konflikt délky payloadu",
                "Aktuální firmware vyžaduje pro 0x08 délku 0 B, ale samotná obsluha čte 1 B. "
                "Tento příkaz může fungovat až po opravě tabulky délek; potom zvolte profil opraveného firmware.",
            )
            return
        if command_id == 0x0E and not messagebox.askyesno(
            "Potvrdit referencování",
            "Motor pojede pevný počet kroků směrem k mechanickému dorazu. Firmware nepoužívá koncový spínač. Pokračovat?",
        ):
            return
        if command_id == 0x0F and int(values["steps"]) > 1000 and not messagebox.askyesno(
            "Potvrdit dlouhý pohyb",
            f"Opravdu odeslat pohyb {int(values['steps'])} kroků?",
        ):
            return
        if command_id == 0x13 and values["light"] == 2 and values["action"] == 1 and not messagebox.askyesno(
            "Potvrdit zapnutí UV",
            "Zapínáte UV Bio-killer. Je zařízení zakrytované a prostor bezpečný?",
        ):
            return
        sequence = int(self.sequence_var.get().strip(), 0)
        if self._write_frame(wire, f"CMD 0x{self.current_command.command_id:02X}, SEQ {sequence}") and self.auto_sequence_var.get():
            self.sequence_var.set(str((sequence + 1) & 0xFF))
            self.update_preview()

    def send_ping(self) -> None:
        try:
            sequence = int(self.sequence_var.get().strip(), 0)
            _raw, wire = build_command(0x0D, sequence, b"", self.profile_var.get())
        except Exception as exc:
            messagebox.showerror("PING nelze sestavit", str(exc))
            return
        if self._write_frame(wire, f"PING / HELLO 0x0D, SEQ {sequence}") and self.auto_sequence_var.get():
            self.sequence_var.set(str((sequence + 1) & 0xFF))
            self.update_preview()

    def send_raw_hex(self) -> None:
        try:
            cleaned = self.raw_hex_var.get().replace("0x", "").replace(",", " ").replace(";", " ")
            wire = bytes.fromhex(cleaned)
            if not wire:
                raise ValueError("Pole je prázdné.")
        except ValueError as exc:
            messagebox.showerror("Neplatný RAW HEX", f"Použijte dvojice hexadecimálních číslic oddělené mezerami.\n\n{exc}")
            return
        self._write_frame(wire, "RAW HEX")

    def send_emergency_stop(self) -> None:
        if self.random_running:
            self._halt_random_motion("NOUZOVÝ STOP", write_log=True)
            self.random_next_var.set("—")
        try:
            sequence = int(self.sequence_var.get().strip(), 0)
            raw, wire = build_command(0x07, sequence, bytes((3, 0)), self.profile_var.get())
            del raw
        except Exception as exc:
            messagebox.showerror("STOP nelze sestavit", str(exc))
            return
        if self._write_frame(wire, f"STOP VŠE, SEQ {sequence}") and self.auto_sequence_var.get():
            self.sequence_var.set(str((sequence + 1) & 0xFF))
            self.update_preview()

    def _log(self, tag: str, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{stamp}] {tag:<4} {message}\n", tag)
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > 2500:
            self.log.delete("1.0", "501.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _on_close(self) -> None:
        self._halt_random_motion("ZASTAVENO", write_log=False)
        self.disconnect()
        self.destroy()


def default_values(command: CommandSpec) -> dict[str, int | float]:
    return {field.key: field.default for field in command.fields}


def run_self_test() -> None:
    expected_lengths = {
        0x00: 1, 0x01: 10, 0x02: 27, 0x03: 9, 0x04: 25, 0x05: 1, 0x06: 15,
        0x07: 2, 0x08: 0, 0x09: 18, 0x0A: 3, 0x0B: 3, 0x0C: 19,
        0x0D: 0, 0x0E: 1, 0x0F: 4, 0x10: 1, 0x11: 1, 0x12: 1, 0x13: 2,
        0x14: 3, 0x15: 1, 0x16: 1, 0x17: 2, 0x18: 1,
    }
    assert expected_lengths == FIRMWARE_PAYLOAD_LENGTHS
    for command in COMMANDS:
        payload = encode_payload(command, default_values(command))
        assert len(payload) == expected_lengths[command.command_id], (command.command_id, len(payload))
        for profile in (PROFILE_CURRENT, PROFILE_FIXED):
            raw, wire = build_command(command.command_id, 0x2A, payload, profile)
            assert raw[0] == command.command_id and raw[1] == 0x2A
            expected_wire_length = (
                FIRMWARE_PAYLOAD_LENGTHS[command.command_id]
                if profile == PROFILE_CURRENT else len(payload)
            )
            assert int.from_bytes(raw[2:4], "big") == expected_wire_length
            assert len(raw) == 2 + 2 + expected_wire_length + 4
            parser = FrameStreamParser()
            assert parser.feed(wire) == [raw]

    special = bytes((0x01, 0x7E, 0x7D, 0x02))
    parser = FrameStreamParser()
    assert parser.feed(escape_body(special)) == [special]
    _raw, stop = build_command(0x07, 0, bytes((3, 0)), PROFILE_CURRENT)
    assert parser.feed(stop) == [_raw]

    # Repeated/split delimiters are boundaries, not zero-byte frames.
    split_parser = FrameStreamParser()
    assert split_parser.feed(b"\x7E") == []
    assert split_parser.feed(b"\x7E") == []
    assert split_parser.feed(_raw + b"\x7E") == [_raw]

    # Reálný start zařízení: 8 self-test notifikací, neorámovaný READY text
    # a následná standardní odpověď mohou dorazit v jediném nebo děleném čtení.
    notifications = b"".join(
        escape_body(bytes((0xFF, status, 0x00, 0x04, 0x00, 0x60, 0x0D, 0x00)) + CRC_PLACEHOLDER)
        for status in range(0x22, 0x26)
    ) + b"".join(
        escape_body(bytes((0xFF, status, 0x00, 0x04, 0x00, 0x60, 0x0D, 0x00)) + CRC_PLACEHOLDER)
        for status in range(0x2A, 0x2E)
    )
    hello_body = bytes((0x8D, 0x2A, 0x77, 0x00, 0x05)) + b"Hello" + CRC_PLACEHOLDER
    mixed_stream = notifications + b"STM32 ready!!!" + escape_body(hello_body)
    parser = FrameStreamParser()
    parsed_frames: list[bytes] = []
    outside = bytearray()
    position = 0
    for chunk_size in (1, 7, 3, 19, 2, 31, 5, 64, 11, 128, 256):
        chunk = mixed_stream[position:position + chunk_size]
        position += len(chunk)
        parsed_frames.extend(parser.feed(chunk))
        outside.extend(parser.drain_outside())
        if position >= len(mixed_stream):
            break
    if position < len(mixed_stream):
        parsed_frames.extend(parser.feed(mixed_stream[position:]))
        outside.extend(parser.drain_outside())
    assert len(parsed_frames) == 9
    assert parsed_frames[-1] == hello_body
    assert b"STM32 ready!!!" in outside
    assert "Hello" in decode_response(parsed_frames[-1])
    assert tmp100_celsius(0x190) == 25.0
    assert tmp100_celsius(0xE70) == -25.0
    assert mmc5603_celsius(100) == 5.0
    assert crc32_mpeg2(b"123456789") == 0x0376E6E7
    assert crc32_mpeg2(bytes((0x87, 0x00, 0x77, 0x00, 0x00))) == 0xE4D8A8F1
    assert frame_crc_ok(bytes((0x87, 0x00, 0x77, 0x00, 0x00, 0xE4, 0xD8, 0xA8, 0xF1)))
    assert not frame_crc_ok(bytes((0x87, 0x00, 0x77, 0x00, 0x00, 0xF1, 0xA8, 0xD8, 0xE4)))

    random_payload = encode_payload(COMMAND_BY_ID[0x06], {
        "azimuth": 180,
        "elevation": -30,
        "amplitude": 100,
        "frequency": 1,
        "offset_x": 0,
        "offset_y": 0,
        "offset_z": 0,
        "ramp": 10,
    })
    assert len(random_payload) == 15
    assert random_payload[0:2] == b"\x00\xB4"
    assert random_payload[2:4] == b"\xFF\xE2"
    random_raw, random_wire = build_command(0x06, 0x2A, random_payload, PROFILE_CURRENT)
    assert int.from_bytes(random_raw[2:4], "big") == 15
    assert FrameStreamParser().feed(random_wire) == [random_raw]
    print(f"Self-test OK: {len(COMMANDS)} příkazů, framing, startovací notifikace a Random Motion.")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="NanoRobot Control Center")
    parser.add_argument("--self-test", action="store_true", help="Ověří payloady a framing bez spuštění GUI.")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.self_test:
        run_self_test()
        return 0
    app = CoilConsole()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
