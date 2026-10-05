@echo off
setlocal
cd /d "%~dp0host\magcal"
where pyw >nul 2>nul
if %errorlevel%==0 (
  start "" pyw -3 "magcal_view.pyw" "..\recordings"
) else (
  start "" pythonw "magcal_view.pyw" "..\recordings"
)
