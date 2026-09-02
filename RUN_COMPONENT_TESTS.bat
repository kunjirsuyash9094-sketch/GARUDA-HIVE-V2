@echo off
title GARUDA-HL-01 MODULAR COMPONENT VALIDATION SUITE
echo ========================================================
echo  GARUDA-HL-01 MODULAR 3D ASSET VALIDATION MENU
echo ========================================================
echo  [1] Validate Entire Assembled Vehicle (GARUDA_MODEL_TEST.tscn)
echo  [2] Validate Rotor Module (ROTOR_TEST.tscn - 0..3000 RPM, CW/CCW)
echo  [3] Validate Gimbal Module (GIMBAL_TEST.tscn - 3-Axis Articulation)
echo  [4] Validate Central Airframe (BODY_TEST.tscn - Faceted Hull)
echo  [5] Validate Master Arm Module (ARM_TEST.tscn - Carbon Tube & Mounts)
echo  [6] Validate Master Motor Module (MOTOR_TEST.tscn - 6215 & CNC Red Ring)
echo  [7] Re-Run Full Python Generation Pipeline
echo ========================================================
set /p opt="Enter choice [1-7]: "

cd /d "c:\GARUDA-HIVE-V2"

if "%opt%"=="1" start "" "godot.exe" --path "c:\GARUDA-HIVE-V2" "res://scenes/GARUDA_MODEL_TEST.tscn"
if "%opt%"=="2" start "" "godot.exe" --path "c:\GARUDA-HIVE-V2" "res://scenes/component_tests/ROTOR_TEST.tscn"
if "%opt%"=="3" start "" "godot.exe" --path "c:\GARUDA-HIVE-V2" "res://scenes/component_tests/GIMBAL_TEST.tscn"
if "%opt%"=="4" start "" "godot.exe" --path "c:\GARUDA-HIVE-V2" "res://scenes/component_tests/BODY_TEST.tscn"
if "%opt%"=="5" start "" "godot.exe" --path "c:\GARUDA-HIVE-V2" "res://scenes/component_tests/ARM_TEST.tscn"
if "%opt%"=="6" start "" "godot.exe" --path "c:\GARUDA-HIVE-V2" "res://scenes/component_tests/MOTOR_TEST.tscn"
if "%opt%"=="7" (
    python "scripts/pipeline/run_full_pipeline.py"
    pause
)
