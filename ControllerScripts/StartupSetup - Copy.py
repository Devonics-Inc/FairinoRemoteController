@echo off

timeout /t 5 /nobreak


Get-WmiObject -Query "SELECT * FROM Win32_USBHub" | ForEach-Object { $_.Disable}

timeout /t 5 /nobreak

Get-WmiObject -Query "SELECT * FROM Win32_USBHub" | ForEach-Object { $_.Enable}

timeout /t 5 /nobreak

python C:\Users\FrTest\Desktop\FrController\ControllerScripts\FrControllerCode.py
