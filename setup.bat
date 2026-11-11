@echo off
rem Download Renode into .renode\ next to this script, where g1sim.py looks for
rem it. Windows x64 only; elsewhere install Renode 1.16+ yourself and put it on
rem PATH or in %RENODE%. Run again to reinstall.
rem
rem Needs curl, certutil and tar (all ship with Windows 10 1803 and later).
setlocal EnableExtensions EnableDelayedExpansion

set "VERSION=1.16.1"
set "URL=https://github.com/renode/renode/releases/download/v%VERSION%/renode-%VERSION%.windows-portable-dotnet.zip"
set "SHA256=d09b7934cfd560cd06bde8f131ef78f521f10d423d5aac6096f2a583224aeb3e"

cd /d "%~dp0" || exit /b 1

set "ARCH=%PROCESSOR_ARCHITECTURE%"
if defined PROCESSOR_ARCHITEW6432 set "ARCH=%PROCESSOR_ARCHITEW6432%"
if /i not "%ARCH%"=="AMD64" (
    >&2 echo setup.bat: no pinned Renode build for Windows %ARCH%.
    >&2 echo Install Renode %VERSION% or newer from https://renode.io and set %%RENODE%%.
    exit /b 1
)

set "tmp=.renode-download.%RANDOM%%RANDOM%"
if exist "%tmp%" rmdir /s /q "%tmp%"
mkdir "%tmp%" || exit /b 1

echo setup.bat: downloading %URL%
curl --fail --location --progress-bar --output "%tmp%\renode.zip" "%URL%"
if errorlevel 1 goto :fail

set "actual="
for /f "skip=1 delims=" %%H in ('certutil -hashfile "%tmp%\renode.zip" SHA256') do (
    if not defined actual set "actual=%%H"
)
set "actual=!actual: =!"
if /i not "!actual!"=="%SHA256%" (
    >&2 echo setup.bat: checksum mismatch, expected %SHA256%, got:
    >&2 echo !actual!  %tmp%\renode.zip
    goto :fail
)

rem The archive holds a single top-level directory.
mkdir "%tmp%\renode" || goto :fail
rem Use Windows' own bsdtar: a GNU tar earlier on PATH (Git, MSYS) can't read zips.
"%SystemRoot%\System32\tar.exe" -xf "%tmp%\renode.zip" -C "%tmp%\renode" --strip-components=1
if errorlevel 1 goto :fail
if not exist "%tmp%\renode\renode.exe" (
    >&2 echo setup.bat: unexpected archive layout, no 'renode.exe' launcher inside
    goto :fail
)

if exist .renode rmdir /s /q .renode
move "%tmp%\renode" .renode >nul || goto :fail
rmdir /s /q "%tmp%"
echo setup.bat: Renode %VERSION% installed in %CD%\.renode
exit /b 0

:fail
if exist "%tmp%" rmdir /s /q "%tmp%"
exit /b 1
