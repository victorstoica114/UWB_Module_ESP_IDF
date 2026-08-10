@echo off
setlocal
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0idf_build.ps1" %*
exit /b %ERRORLEVEL%
