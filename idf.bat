@echo off
setlocal

set "IDF_PATH=C:\esp\v6.0.2\esp-idf"
set "IDF_TOOLS_PATH=C:\Espressif\tools"
set "IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v6.0.2\venv"
set "ESP_IDF_VERSION=6.0"
set "IDF_COMPONENT_LOCAL_STORAGE_URL=file://C:\Espressif\tools"
set "ESP_ROM_ELF_DIR=C:\Espressif\tools\esp-rom-elfs\20241011"
set "OPENOCD_SCRIPTS=C:\Espressif\tools\openocd-esp32\v0.12.0-esp32-20260424\openocd-esp32\share\openocd\scripts"

set "PATH=C:\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64;C:\Espressif\tools\cmake\4.0.3\bin;C:\Espressif\tools\idf-exe\1.0.3;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\openocd-esp32\v0.12.0-esp32-20260424\openocd-esp32\bin;C:\Espressif\tools\xtensa-esp-elf-gdb\17.1_20260402\xtensa-esp-elf-gdb\bin;C:\Espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin;C:\Espressif\tools\python\v6.0.2\venv\Scripts;%PATH%"

"%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" "%IDF_PATH%\tools\idf.py" %*
