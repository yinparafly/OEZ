@echo off
dism /online /enable-feature /featurename:Microsoft-Hyper-V-Tools-All /all /norestart > D:\oezcon\dism_output.txt 2>&1
echo Done >> D:\oezcon\dism_output.txt
