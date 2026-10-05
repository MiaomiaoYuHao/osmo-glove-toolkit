@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0.."
where pyw >nul 2>nul
if %errorlevel%==0 (
  start "" pyw -3 "姿态测试上位机.pyw"
) else (
  start "" pythonw "姿态测试上位机.pyw"
)
