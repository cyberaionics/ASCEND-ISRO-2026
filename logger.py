"""
ASCEND Phase 2 — Logger
Timestamped, ANSI colour-coded console logger with severity levels
and state-transition banners.
"""

import datetime


class Logger:
    """ANSI-coloured, timestamped console logger.

    Provides ``info``, ``ok``, ``warn``, ``error`` severity levels,
    section/header formatting, and a dedicated ``state()`` method
    that prints a prominent banner on flight-state transitions.
    """

    # ANSI colour codes
    _RESET   = "\033[0m"
    _BOLD    = "\033[1m"
    _DIM     = "\033[2m"
    _CYAN    = "\033[96m"
    _GREEN   = "\033[92m"
    _YELLOW  = "\033[93m"
    _RED     = "\033[91m"
    _MAGENTA = "\033[95m"
    _WHITE   = "\033[97m"

    _TAGS = {
        "INFO":  "\033[96m[INFO ]\033[0m",
        "OK":    "\033[92m[ OK  ]\033[0m",
        "WARN":  "\033[93m[WARN ]\033[0m",
        "ERROR": "\033[91m[ERROR]\033[0m",
    }

    # ── Core ───────────────────────────────────────────────────────────

    @staticmethod
    def _ts() -> str:
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    @classmethod
    def _log(cls, level: str, msg: str) -> None:
        tag = cls._TAGS.get(level, cls._TAGS["INFO"])
        print(f"{cls._DIM}{cls._ts()}{cls._RESET} {tag} {msg}")

    # ── Public Levels ──────────────────────────────────────────────────

    @classmethod
    def info(cls, msg: str) -> None:
        """Log an informational message."""
        cls._log("INFO", msg)

    @classmethod
    def ok(cls, msg: str) -> None:
        """Log a success message."""
        cls._log("OK", msg)

    @classmethod
    def warn(cls, msg: str) -> None:
        """Log a warning."""
        cls._log("WARN", msg)

    @classmethod
    def error(cls, msg: str) -> None:
        """Log an error."""
        cls._log("ERROR", msg)

    # ── State Transition Banner ────────────────────────────────────────

    @classmethod
    def state(cls, from_state: str, to_state: str, reason: str = "") -> None:
        """Print a prominent banner for a flight-state transition.

        Args:
            from_state: Name of the state being left.
            to_state:   Name of the state being entered.
            reason:     Optional human-readable reason for the transition.
        """
        line = "═" * 56
        tag = f"{from_state} → {to_state}"
        print(f"\n{cls._BOLD}{cls._MAGENTA}{line}{cls._RESET}")
        print(f"{cls._BOLD}{cls._WHITE}  STATE: {tag}{cls._RESET}")
        if reason:
            print(f"{cls._DIM}  reason: {reason}{cls._RESET}")
        print(f"{cls._BOLD}{cls._MAGENTA}{line}{cls._RESET}\n")

    # ── Formatting ─────────────────────────────────────────────────────

    @classmethod
    def header(cls, title: str) -> None:
        """Print a section header."""
        line = "━" * 60
        print(f"\n{cls._BOLD}{cls._MAGENTA}{line}{cls._RESET}")
        print(f"{cls._BOLD}{cls._WHITE}  {title}{cls._RESET}")
        print(f"{cls._BOLD}{cls._MAGENTA}{line}{cls._RESET}")

    @classmethod
    def section(cls, title: str) -> None:
        """Print a sub-section divider."""
        print(f"\n{cls._BOLD}{cls._CYAN}── {title} ──{cls._RESET}")

    @classmethod
    def kv(cls, key: str, value: object, pad: int = 24) -> None:
        """Print a key-value pair with alignment."""
        print(f"  {cls._WHITE}{key:<{pad}}{cls._RESET} : {value}")

    @classmethod
    def banner(cls) -> None:
        """Print the ASCEND startup banner."""
        b = [
            "",
            f"{cls._BOLD}{cls._CYAN}"
            "     █████╗ ███████╗ ██████╗███████╗███╗   ██╗██████╗ ",
            "    ██╔══██╗██╔════╝██╔════╝██╔════╝████╗  ██║██╔══██╗",
            "    ███████║███████╗██║     █████╗  ██╔██╗ ██║██║  ██║",
            "    ██╔══██║╚════██║██║     ██╔══╝  ██║╚██╗██║██║  ██║",
            "    ██║  ██║███████║╚██████╗███████╗██║ ╚████║██████╔╝",
            "    ╚═╝  ╚═╝╚══════╝ ╚═════╝╚══════╝╚═╝  ╚═══╝╚═════╝ "
            f"{cls._RESET}",
            f"    {cls._DIM}Phase 2 · Autonomous Flight · Bayes Frontier{cls._RESET}",
            "",
        ]
        print("\n".join(b))
