@echo off
rem ABI 监控 PC UI 一键启动 - USB 串口模式（自动选第一个可用 COM）
cd /d "%~dp0"
start "" python -X utf8 abi_monitor.py --usb