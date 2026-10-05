@echo off
setlocal
cd /d "%~dp0host"
where pyw >nul 2>nul
if %errorlevel%==0 (
  start "" pyw -3 "纯数据预览.pyw"
) else (
  start "" pythonw "纯数据预览.pyw"
)