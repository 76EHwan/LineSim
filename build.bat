@echo off
rem Configure + build LineSim with the Visual Studio 2022 toolchain.
rem   build.bat            -> Release build of everything
rem   build.bat Debug      -> Debug build
rem   build.bat Release firmware fw_shim  -> specific targets
setlocal
set "PATH=%SystemRoot%\System32;%SystemRoot%;%ProgramFiles(x86)%\Microsoft Visual Studio\Installer;%PATH%"
set CONFIG=%1
if "%CONFIG%"=="" set CONFIG=Release
set "CMAKE=cmake"
where cmake >nul 2>nul || set "CMAKE=%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
cd /d %~dp0
if not exist build\CMakeCache.txt (
  "%CMAKE%" -S . -B build -G "Visual Studio 17 2022" -A x64 %LINESIM_CMAKE_ARGS% || exit /b 1
)
shift
set TARGETS=
:collect
if "%1"=="" goto run
set TARGETS=%TARGETS% --target %1
shift
goto collect
:run
"%CMAKE%" --build build --config %CONFIG% %TARGETS% -- /nologo /v:minimal /m /nr:false || exit /b 1

