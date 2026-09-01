@echo off
setlocal
cd /d "%~dp0\.."
set CONFIG=%1
if "%CONFIG%"=="" set CONFIG=Release
echo [Garuda Hive] Configuring (%CONFIG%)...
cmake -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=%CONFIG%
if errorlevel 1 goto :err
echo [Garuda Hive] Building...
cmake --build build --config %CONFIG% --parallel
if errorlevel 1 goto :err
echo [Garuda Hive] Done -^> gdextension\bin\
goto :eof
:err
echo [Garuda Hive] BUILD FAILED
exit /b 1
