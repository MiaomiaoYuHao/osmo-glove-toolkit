@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0..\magcal"
where pyw >nul 2>nul
if %errorlevel%==0 (
  start "" pyw -3 "magcal_view.pyw" "..\recordings"
) else (
  start "" pythonw "magcal_view.pyw" "..\recordings"
)
