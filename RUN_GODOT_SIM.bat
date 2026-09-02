@echo off
title GARUDA HIVE V2 - Godot 4 Simulator
echo ============================================================
echo   GARUDA HIVE V2 - Godot 4 Autonomous Drone Simulator
echo ============================================================
echo [*] Launching Godot 4 Simulation...
cd /d "%~dp0"
start "" "godot.exe" --path .
echo [+] Godot 4 Simulation Launched!
