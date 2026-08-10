@echo off
setlocal

rem Backward-compatible entry point for plain CMD and VS Code terminals.
rem idf_build.ps1 discovers ESP-IDF, keeps ccache in the repository, and adds
rem process-local Git safe.directory exceptions without changing global state.
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\idf_build.ps1" -BuildDir build %*
exit /b %ERRORLEVEL%
