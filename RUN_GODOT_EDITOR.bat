@echo off
title GARUDA HIVE V2 - Godot 4 Editor
echo ============================================================
echo   GARUDA HIVE V2 - Godot 4 Visual Project Editor
echo ============================================================
echo [*] Launching Godot 4 Editor...
cd /d "%~dp0"
start "" "godot.exe" --path . --editor
echo [+] Godot 4 Editor Launched!
