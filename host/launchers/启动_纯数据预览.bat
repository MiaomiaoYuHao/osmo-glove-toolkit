@echo off
chcp 936 >nul
setlocal
cd /d "%~dp0.."
where pyw >nul 2>nul
if %errorlevel%==0 (
  start "" pyw -3 "纯数据预览.pyw"
) else (
  start "" pythonw "纯数据预览.pyw"
)
