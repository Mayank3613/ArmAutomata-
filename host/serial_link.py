"""
Serial link to the Arduino — sends motor command packets over USB serial.

Packet format (ASCII, newline-terminated) for the 6-DOF arm:
    base_spd,shoulder_spd,elbow_spd,wrist_rot,wrist_ext,claw\n

- base_spd, shoulder_spd, elbow_spd: integers in [-100, 100] (continuous rotation speeds; 0 = stop).
- wrist_rot, wrist_ext, claw: integers in [0, 180] (servo angles in degrees).
"""

from __future__ import annotations

import time
from typing import Optional

import serial  # pyserial

from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_RESET_WAIT_S


class SerialLink:
    """Thin wrapper over pyserial for communicating with the arm firmware.

    Args:
        port: Serial port path (e.g. ``/dev/ttyUSB0``, ``COM3``).
        baud: Baud rate (must match the Arduino sketch).
        reset_wait: Seconds to wait after opening the port so the Arduino
                    has time to reset (the DTR toggle causes a reboot).
    """

    def __init__(
        self,
        port: str = SERIAL_PORT,
        baud: int = SERIAL_BAUD,
        reset_wait: float = SERIAL_RESET_WAIT_S,
    ) -> None:
        self.port = port
        self.baud = baud
        self.reset_wait = reset_wait
        self._ser: Optional[serial.Serial] = None

    def open(self) -> None:
        """Open the serial port and wait for the Arduino to reset.

        Raises:
            RuntimeError: If the port cannot be opened.
        """
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1)
        except serial.SerialException as exc:
            raise RuntimeError(
                f"Cannot open serial port {self.port} @ {self.baud} baud. "
                f"Check the port name and that the Arduino is connected.\n"
                f"  Detail: {exc}"
            ) from exc

        # Wait for the Arduino's bootloader reset
        print(
            f"[serial_link] Waiting {self.reset_wait:.1f}s for Arduino reset…"
        )
        time.sleep(self.reset_wait)
        # Flush any boot messages
        if self._ser.in_waiting:
            self._ser.read(self._ser.in_waiting)
        print("[serial_link] Ready.")

    def send_angles(
        self,
        wrist_rot: int,
        wrist_ext: int,
        claw: int,
    ) -> None:
        """Send a motor command packet to the Arduino.

        MG996R continuous motors (Ch 0-2) are always sent speed 0 (stopped).
        Only the 3 positional micro-servos are controlled.

        Args:
            wrist_rot: Wrist rotation angle (0–180).
            wrist_ext: Wrist extension angle (0–180).
            claw: Claw (gripper) angle (0–180).
        """
        if self._ser is None or not self._ser.is_open:
            return
        packet = f"0,0,0,{wrist_rot},{wrist_ext},{claw}\n"
        self._ser.write(packet.encode("ascii"))

    def close(self) -> None:
        """Close the serial port."""
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            print("[serial_link] Port closed.")

    # -- context-manager support --
    def __enter__(self) -> "SerialLink":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
