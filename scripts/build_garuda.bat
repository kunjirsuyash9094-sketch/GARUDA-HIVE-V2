@echo off
setlocal enabledelayedexpansion

echo ============================================================
echo  GARUDA HIVE V2 — STANDALONE C++20 COMPILATION SCRIPT
echo ============================================================

set "PATH=C:\msys64\ucrt64\bin;%PATH%"

echo [1/4] Compiling garuda_physics.dll (C-ABI Shared Library)...
g++ -std=c++20 -O3 -shared -static-libgcc -static-libstdc++ -DGARUDA_EXPORTS -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp src/garuda/capi.cpp -o garuda_physics.dll "-Wl,--out-implib,libgaruda_physics.a"
if %ERRORLEVEL% NEQ 0 (echo Compilation failed! && exit /b 1)

echo [2/4] Compiling garuda_sim.exe (Headless CLI Simulator)...
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp src/garuda_sim.cpp -o garuda_sim.exe
if %ERRORLEVEL% NEQ 0 (echo Compilation failed! && exit /b 1)

echo [3/4] Compiling Unit Tests...
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp tests/test_phase1_physics.cpp -o test_phase1_physics.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/battery_model.cpp tests/test_battery.cpp -o test_battery.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include tests/test_motor.cpp -o test_motor.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp tests/test_ground_contact.cpp -o test_ground_contact.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp tests/test_multidrone.cpp -o test_multidrone.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp tests/test_payload.cpp -o test_payload.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include tests/test_sensor_suite.cpp -o test_sensor_suite.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include tests/test_inspection_camera.cpp -o test_inspection_camera.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/battery_model.cpp tests/test_health.cpp -o test_health.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp tests/test_phase2_integration.cpp -o test_phase2_integration.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp tests/test_determinism.cpp -o test_determinism.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp tests/test_replay.cpp -o test_replay.exe
g++ -std=c++20 -O3 -static-libgcc -static-libstdc++ -I include src/garuda/physics/ground_contact.cpp src/garuda/physics/battery_model.cpp src/garuda/physics/rigid_body.cpp src/garuda/core/drone_instance.cpp src/garuda/core/simulation_world.cpp tests/test_benchmarks.cpp -o test_benchmarks.exe

echo [4/4] ALL TARGETS COMPILED SUCCESSFULLY.
echo ============================================================
