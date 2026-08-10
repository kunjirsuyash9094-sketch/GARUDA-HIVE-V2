@echo off
setlocal
cd /d "%~dp0\.."
set CONFIG=%1
if "%CONFIG%"=="" set CONFIG=Release
echo [SkySim] Configuring (%CONFIG%)...
cmake -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=%CONFIG%
if errorlevel 1 goto :err
echo [SkySim] Building...
cmake --build build --config %CONFIG% --parallel
if errorlevel 1 goto :err
echo [SkySim] Done -^> gdextension\bin\
goto :eof
:err
echo [SkySim] BUILD FAILED
exit /b 1
