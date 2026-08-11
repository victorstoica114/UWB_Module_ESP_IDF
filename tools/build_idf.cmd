@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_idf.ps1" %*
exit /b %ERRORLEVEL%
