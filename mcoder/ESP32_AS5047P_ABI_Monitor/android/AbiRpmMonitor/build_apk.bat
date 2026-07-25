@echo off
REM Build AbiRpmMonitor debug APK and copy to mcoder\
set JAVA_HOME=D:\Android\AndroidStudio\jbr
set PATH=%JAVA_HOME%\bin;%PATH%
cd /d "%~dp0"
call gradlew.bat assembleDebug
if errorlevel 1 exit /b 1
copy /Y "app\build\outputs\apk\debug\app-debug.apk" "E:\OEZCON\mcoder\AbiRpmMonitor-debug.apk"
echo.
echo OK: E:\OEZCON\mcoder\AbiRpmMonitor-debug.apk
echo Install: adb install -r E:\OEZCON\mcoder\AbiRpmMonitor-debug.apk
exit /b 0
