"""Explicit, fail-closed operator selection for ZED body tracking."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Iterable


class OperatorState(str, Enum):
    WAITING = "WAITING"
    ACQUIRING = "ACQUIRING"
    LOCKED = "LOCKED"
    LOST = "LOST"


@dataclass(frozen=True)
class OperatorSelection:
    body: Any | None
    body_id: int | None
    unique_object_id: str | None
    state: OperatorState
    missing_frames: int
    acquisition_frames: int
    reason: str


class OperatorSelector:
    """Lock one ZED identity and never silently transfer control.

    The nearest valid candidate must remain the same for ``acquire_frames``.
    Once locked, disappearance produces LOST while preserving the selected ID.
    Only :meth:`reset` permits a different person to become operator.
    """

    def __init__(self, acquire_frames: int = 10) -> None:
        self.acquire_frames = max(1, int(acquire_frames))
        self.locked_id: int | None = None
        self.locked_unique_id: str | None = None
        self._candidate_id: int | None = None
        self._candidate_unique_id: str | None = None
        self._candidate_frames = 0
        self._missing_frames = 0

    def reset(self) -> None:
        self.locked_id = None
        self.locked_unique_id = None
        self._candidate_id = None
        self._candidate_unique_id = None
        self._candidate_frames = 0
        self._missing_frames = 0

    @staticmethod
    def _identity(body: Any) -> tuple[int, str]:
        raw_unique_id = getattr(body, "unique_object_id", "")
        text = str(raw_unique_id).strip() if raw_unique_id is not None else ""
        # Some PyZED builds expose unique_object_id through a temporary wrapper
        # whose default repr contains its Python memory address.  That address
        # changes on every retrieve_bodies() call and must not be treated as a
        # person identity.  ZED's integer body.id is the documented fixed
        # tracking ID and remains the primary acquisition key.
        if text.startswith("<") and " object at 0x" in text:
            text = ""
        return int(body.id), text

    def update(
        self,
        bodies: Iterable[Any],
        *,
        valid: Callable[[Any], bool],
        distance: Callable[[Any], float],
    ) -> OperatorSelection:
        candidates = [body for body in bodies if valid(body)]
        if self.locked_id is not None:
            for body in candidates:
                body_id, unique_id = self._identity(body)
                id_ok = body_id == self.locked_id
                uid_ok = (
                    not self.locked_unique_id
                    or not unique_id
                    or unique_id == self.locked_unique_id
                )
                if id_ok and uid_ok:
                    self._missing_frames = 0
                    return OperatorSelection(
                        body, self.locked_id, self.locked_unique_id,
                        OperatorState.LOCKED, 0, self.acquire_frames, "operator_locked",
                    )
            self._missing_frames += 1
            return OperatorSelection(
                None, self.locked_id, self.locked_unique_id,
                OperatorState.LOST, self._missing_frames,
                self.acquire_frames, "locked_operator_missing",
            )

        if not candidates:
            self._candidate_id = None
            self._candidate_unique_id = None
            self._candidate_frames = 0
            return OperatorSelection(
                None, None, None, OperatorState.WAITING, 0, 0, "no_valid_candidate"
            )

        body = min(candidates, key=distance)
        body_id, unique_id = self._identity(body)
        if body_id == self._candidate_id:
            self._candidate_frames += 1
            # Retain the stronger UUID check only when the binding reports a
            # stable value throughout acquisition.  An unstable UUID must not
            # reset an otherwise stable ZED tracking ID every frame.
            if self._candidate_unique_id and unique_id != self._candidate_unique_id:
                self._candidate_unique_id = ""
        else:
            self._candidate_id = body_id
            self._candidate_unique_id = unique_id
            self._candidate_frames = 1

        if self._candidate_frames >= self.acquire_frames:
            self.locked_id = body_id
            self.locked_unique_id = self._candidate_unique_id or ""
            self._missing_frames = 0
            return OperatorSelection(
                body, body_id, self.locked_unique_id, OperatorState.LOCKED, 0,
                self._candidate_frames, "operator_lock_confirmed",
            )
        return OperatorSelection(
            body, body_id, unique_id, OperatorState.ACQUIRING, 0,
            self._candidate_frames, "candidate_stability_check",
        )
