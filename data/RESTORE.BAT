@ECHO OFF

REM --- Always recreate CONFIG.SYS and AUTOEXEC.BAT ---
ECHO Restoring C:\CONFIG.SYS and C:\AUTOEXEC.BAT...

REM --- CONFIG.SYS: use COMMAND.COM as shell, bypass SysMgr ROM fallback ---
ECHO SHELL=D:\DOS\COMMAND.COM /P D:\DOS> C:\CONFIG.SYS

ECHO C:\CONFIG.SYS written:
TYPE C:\CONFIG.SYS
ECHO.

REM --- Recreate C:\AUTOEXEC.BAT ---
ECHO @echo off> C:\AUTOEXEC.BAT
ECHO prompt $p$g>> C:\AUTOEXEC.BAT
ECHO path c:\;d:\;d:\bin;d:\dos>> C:\AUTOEXEC.BAT
ECHO c:>> C:\AUTOEXEC.BAT
ECHO assign e:=a:>> C:\AUTOEXEC.BAT
ECHO a:\KBDSER2.COM /G /D /K>> C:\AUTOEXEC.BAT
ECHO A:\PROGRAMS\vc\VC.COM>> C:\AUTOEXEC.BAT

ECHO C:\AUTOEXEC.BAT written:
ECHO -------------------------
TYPE C:\AUTOEXEC.BAT
ECHO -------------------------
ECHO.

REM --- Load KBDSER2 now for current session ---
ECHO Loading KBDSER2...
a:\KBDSER2.COM /U
a:\KBDSER2.COM /G /D /K
ECHO.
a:\KBDSER2.COM /S
ECHO.
ECHO Top-right corner digit changes on each BLE keypress.
ECHO.

REM --- Reboot so CONFIG.SYS + AUTOEXEC.BAT take effect ---
ECHO Rebooting...
ECHO (If hangs: press reset hole on back of HP 200LX)
ECHO g=ffff:0 | DEBUG

:END