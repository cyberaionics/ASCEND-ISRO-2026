"""
ASCEND Phase 1 — Logger
Timestamped, colour-coded console logger with severity levels.
"""

import datetime


class Logger:
    """ANSI-coloured, timestamped console logger.

    Provides ``info``, ``ok``, ``warn``, ``error`` severity levels plus
    convenience helpers for section headers and horizontal rules.
    """

    # ANSI colour codes
    _RESET  = "\033[0m"
    _BOLD   = "\033[1m"
    _DIM    = "\033[2m"
    _CYAN   = "\033[96m"
    _GREEN  = "\033[92m"
    _YELLOW = "\033[93m"
    _RED    = "\033[91m"
    _MAGENTA = "\033[95m"
    _WHITE  = "\033[97m"

    # Severity tag styles
    _TAGS = {
        "INFO":  f"\033[96m[INFO ]\033[0m",
        "OK":    f"\033[92m[ OK  ]\033[0m",
        "WARN":  f"\033[93m[WARN ]\033[0m",
        "ERROR": f"\033[91m[ERROR]\033[0m",
    }

    # ── Core Logging ───────────────────────────────────────────────────

    @staticmethod
    def _timestamp() -> str:
        """Return a compact ISO-8601 timestamp for the current instant."""
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    @classmethod
    def _log(cls, level: str, msg: str) -> None:
        tag = cls._TAGS.get(level, cls._TAGS["INFO"])
        print(f"{cls._DIM}{cls._timestamp()}{cls._RESET} {tag} {msg}")

    # ── Public API ─────────────────────────────────────────────────────

    @classmethod
    def info(cls, msg: str) -> None:
        """Log an informational message (cyan tag)."""
        cls._log("INFO", msg)

    @classmethod
    def ok(cls, msg: str) -> None:
        """Log a success message (green tag)."""
        cls._log("OK", msg)

    @classmethod
    def warn(cls, msg: str) -> None:
        """Log a warning message (yellow tag)."""
        cls._log("WARN", msg)

    @classmethod
    def error(cls, msg: str) -> None:
        """Log an error message (red tag)."""
        cls._log("ERROR", msg)

    # ── Formatting Helpers ─────────────────────────────────────────────

    @classmethod
    def header(cls, title: str) -> None:
        """Print a prominent section header."""
        line = "━" * 60
        print(f"\n{cls._BOLD}{cls._MAGENTA}{line}{cls._RESET}")
        print(f"{cls._BOLD}{cls._WHITE}  {title}{cls._RESET}")
        print(f"{cls._BOLD}{cls._MAGENTA}{line}{cls._RESET}")

    @classmethod
    def section(cls, title: str) -> None:
        """Print a sub-section divider."""
        print(f"\n{cls._BOLD}{cls._CYAN}── {title} ──{cls._RESET}")

    @classmethod
    def rule(cls) -> None:
        """Print a thin horizontal rule."""
        print(f"{cls._DIM}{'─' * 60}{cls._RESET}")

    @classmethod
    def kv(cls, key: str, value: object, pad: int = 24) -> None:
        """Print a key-value pair, left-padded for alignment."""
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
            f"    {cls._DIM}Autonomous Surveyor · Phase 1 · Bayes Frontier{cls._RESET}",
            "",
        ]
        print("\n".join(b))
