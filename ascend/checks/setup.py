"""
ASCEND Phase 1 — Pixhawk Parameter Setup
Writes all parameter groups (rangefinder, autotune, failsafe, flight)
to the Pixhawk with ACK verification.
"""

from ..config import Config
from ..logger import Logger
from ..hardware.pixhawk import PixhawkLink


class AutoTuneSetup:
    """Write all Phase 1 Pixhawk parameters in grouped batches.

    Each parameter is written via ``PixhawkLink.write_param()`` and verified
    with a PARAM_VALUE ACK. Failures are reported but do not abort the
    remaining group.

    Args:
        pixhawk: Connected PixhawkLink instance.
    """

    def __init__(self, pixhawk: PixhawkLink) -> None:
        self._px = pixhawk

    # ── Public API ─────────────────────────────────────────────────────

    def run_all(self) -> bool:
        """Write all parameter groups and return overall success.

        Returns:
            ``True`` if every parameter was written and confirmed.
        """
        Logger.header("PIXHAWK PARAMETER SETUP")

        ok = True
        ok &= self._write_group("Rangefinder", Config.PARAMS_RANGEFINDER)
        ok &= self._write_group("AutoTune (reset PIDs)", Config.PARAMS_AUTOTUNE)
        ok &= self._write_group("Failsafe", Config.PARAMS_FAILSAFE)
        ok &= self._write_group("Flight", Config.PARAMS_FLIGHT)

        Logger.section("Summary")
        if ok:
            Logger.ok("All parameters written successfully ✓")
        else:
            Logger.error("Some parameters FAILED to write — see above")
        return ok

    # ── Internal ───────────────────────────────────────────────────────

    def _write_group(self, group_name: str, params: dict) -> bool:
        """Write a named group of parameters.

        Args:
            group_name: Human-readable group label.
            params: ``{name: value}`` mapping of parameters.

        Returns:
            ``True`` if every parameter in the group was confirmed.
        """
        Logger.section(f"Group: {group_name}")
        all_ok = True
        for name, value in params.items():
            success = self._px.write_param(name, float(value))
            if success:
                Logger.ok(f"  {name} = {value}")
            else:
                Logger.error(f"  {name} = {value}  ← WRITE FAILED")
                all_ok = False
        return all_ok
