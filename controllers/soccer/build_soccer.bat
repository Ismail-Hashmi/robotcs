@echo off
setlocal EnableExtensions
cd /d "%~dp0"

if not defined WEBOTS_HOME (
  if exist "C:\Program Files\Webots" set "WEBOTS_HOME=C:\Program Files\Webots"
)
if not defined WEBOTS_HOME (
  if exist "C:\Program Files (x86)\Webots" set "WEBOTS_HOME=C:\Program Files (x86)\Webots"
)
if not defined WEBOTS_HOME (
  if exist "%LOCALAPPDATA%\Programs\Webots" set "WEBOTS_HOME=%LOCALAPPDATA%\Programs\Webots"
)

if not defined WEBOTS_HOME (
  echo ERROR: WEBOTS_HOME is not set and no Webots folder was found in Program Files.
  echo Set it to your install path, e.g.:
  echo   set WEBOTS_HOME=C:\Program Files\Webots
  exit /b 1
)

set "MAKE_EXE="
if exist "%WEBOTS_HOME%\msys64\usr\bin\make.exe" set "MAKE_EXE=%WEBOTS_HOME%\msys64\usr\bin\make.exe"
if not defined MAKE_EXE if exist "%WEBOTS_HOME%\msys64\mingw64\bin\mingw32-make.exe" set "MAKE_EXE=%WEBOTS_HOME%\msys64\mingw64\bin\mingw32-make.exe"
if not defined MAKE_EXE if exist "%WEBOTS_HOME%\msys\mingw64\bin\mingw32-make.exe" set "MAKE_EXE=%WEBOTS_HOME%\msys\mingw64\bin\mingw32-make.exe"

if not defined MAKE_EXE (
  for /f "delims=" %%M in ('where mingw32-make 2^>nul') do (
    set "MAKE_EXE=%%M"
    goto make_done
  )
)
:make_done

if defined MAKE_EXE goto use_mingw_make

if exist "%WEBOTS_HOME%\msys64\usr\bin\bash.exe" (
  echo mingw32-make not found; using MSYS2 bash + make from Webots.
  set "U=%CD:\=/%"
  set "U=%U:C:=/c%"
  set "W=%WEBOTS_HOME:\=/%"
  set "W=%W:C:=/c%"
  "%WEBOTS_HOME%\msys64\usr\bin\bash.exe" -lc "export WEBOTS_HOME='%W%' && export PATH='/usr/bin:/mingw64/bin:/usr/local/bin:'$PATH && cd '%U%' && make clean && make"
  if errorlevel 1 (
    echo BUILD failed.
    exit /b 1
  )
  goto build_ok
)

echo ERROR: No build tool found under WEBOTS_HOME=%WEBOTS_HOME%
echo Install Webots fully or use menu: Build ^> Clean, then Build ^> Build.
exit /b 1

:use_mingw_make
echo WEBOTS_HOME=%WEBOTS_HOME%
echo Building soccer controller in:
echo   %CD%
echo.

set "PATH=%WEBOTS_HOME%\msys64\usr\bin;%WEBOTS_HOME%\msys64\mingw64\bin;%PATH%"

"%MAKE_EXE%" clean
if errorlevel 1 (
  echo CLEAN failed.
  exit /b 1
)

"%MAKE_EXE%"
if errorlevel 1 (
  echo BUILD failed — fix errors above, then run this script again.
  exit /b 1
)

:build_ok

echo.
echo SUCCESS: soccer.exe rebuilt. In Webots set speed to 1.0x and run the world.
dir /-c soccer.exe 2>nul
exit /b 0
