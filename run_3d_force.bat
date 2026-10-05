@echo off
setlocal
cd /d "%~dp0host"
where pyw >nul 2>nul
if %errorlevel%==0 (
  start "" pyw -3 "force_3d_ui_launcher.pyw"
) else (
  start "" pythonw "force_3d_ui_launcher.pyw"
)