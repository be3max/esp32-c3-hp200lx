@ECHO OFF
REM ============================================================
REM LOAD2.BAT - Reload KBDSER2 (normal or game mode)
REM
REM Put this file AND KBDSER2.COM on A: flash card.
REM Flash card survives battery changes. C: does not.
REM
REM Usage:
REM   LOAD2         - BIOS buffer mode (normal DOS apps, VC)
REM   LOAD2 /G      - GAME mode (8042 port stuffing, for Doom etc)
REM   LOAD2 /G /D   - GAME mode + debug indicator
REM ============================================================

ECHO KBDSER2 loader
ECHO ==============

REM --- Unload previous instance if loaded ---
A:\KBDSER2.COM /U
ECHO.

REM --- Load fresh, passing all args through ---
A:\KBDSER2.COM /G /D /K
ECHO.

REM --- Confirm loaded and show byte counter ---
A:\KBDSER2.COM /S
ECHO.
ECHO Watch top-right corner of screen.
ECHO Press a BLE key - corner digit should change.
ECHO To unload: A:\KBDSER2.COM /U
