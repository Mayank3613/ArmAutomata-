"""
Angle smoothing with exponential moving average (EMA) and deadband.

Prevents jittery servo movement by smoothing incoming joint angles and
suppressing retransmission when angles haven't changed meaningfully.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from config import SMOOTHING_ALPHA, DEADBAND_DEGREES


class AngleSmoother:
    """EMA smoother with deadband for N joint channels.

    For the 6-DOF arm, use ``num_channels=5`` to smooth the five continuous
    arm angles (base, shoulder, elbow, wrist_rotation, wrist_extension).
    The claw angle is smoothed separately via the same mechanism.

    Args:
        num_channels: Number of joint angles to smooth.
        alpha: EMA weight for new readings.  Higher = less smoothing.
        deadband: Minimum change in any channel (degrees) required to
                  consider the output "new".
    """

    def __init__(
        self,
        num_channels: int = 5,
        alpha: float = SMOOTHING_ALPHA,
        deadband: float = DEADBAND_DEGREES,
    ) -> None:
        self.num_channels = num_channels
        self.alpha = alpha
        self.deadband = deadband

        self._state: Optional[List[float]] = None
        self._last_sent: Optional[List[float]] = None

    def update(
        self, angles: Tuple[float, ...] | List[float]
    ) -> Tuple[Optional[Tuple[float, ...]], bool]:
        """Feed new raw angles and get smoothed output.

        Args:
            angles: Raw joint angles (length must equal *num_channels*).

        Returns:
            (smoothed_angles, changed)
            *smoothed_angles* is the EMA-filtered tuple (always returned),
            *changed* is ``True`` only when at least one channel has moved
            beyond the deadband since the last ``changed=True`` result.
        """
        if len(angles) != self.num_channels:
            raise ValueError(
                f"Expected {self.num_channels} angles, got {len(angles)}"
            )

        # Initialise state on first call
        if self._state is None:
            self._state = list(angles)
            self._last_sent = list(angles)
            return tuple(self._state), True

        # EMA update
        for i in range(self.num_channels):
            self._state[i] = (
                self.alpha * angles[i] + (1.0 - self.alpha) * self._state[i]
            )

        smoothed = tuple(self._state)

        # Deadband check
        assert self._last_sent is not None
        changed = any(
            abs(self._state[i] - self._last_sent[i]) > self.deadband
            for i in range(self.num_channels)
        )

        if changed:
            self._last_sent = list(self._state)

        return smoothed, changed

    def reset(self) -> None:
        """Clear internal state so the next ``update`` re-initialises."""
        self._state = None
        self._last_sent = None
