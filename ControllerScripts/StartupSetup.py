@echo off

timeout /t 10 /nobreak

Get-WmiObject Win32_PnPEntity | Where-Object { $_.DeviceID -like "USB*"}

timeout /t 10 /nobreak

python C:\Users\FrTest\Desktop\FrController\ControllerScripts\FrControllerCode.py