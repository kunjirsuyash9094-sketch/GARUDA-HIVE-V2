@echo off
title GARUDA-HL-01 3D MODEL MECHANICAL VALIDATION
echo ========================================================
echo  GARUDA-HL-01 MECHANICAL 3D ASSET VALIDATION TEST
echo  Orthographic Views, 8x CW/CCW Rotors, 3-Axis Gimbal, Scale
echo ========================================================
cd /d "%~dp0"

taskkill /F /IM godot.exe >nul 2>&1
taskkill /F /IM godot_console.exe >nul 2>&1
ping 127.0.0.1 -n 2 >nul

echo Launching Godot 4 GARUDA-HL-01 Model Test Scene...
start "" "%~dp0godot.exe" --path "%~dp0." "res://scenes/GARUDA_MODEL_TEST.tscn"
echo [OK] Test scene active on screen.
