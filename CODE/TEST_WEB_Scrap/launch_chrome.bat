@echo off
title Launching Chrome for Gemini Web Automation
echo ===================================================================
echo  🚀 Starting Google Chrome with Remote Debugging on Port 9222...
echo ===================================================================
echo.
echo 1. Chrome will open to https://gemini.google.com
echo 2. Sign in to your Google / Gemini account in that window.
echo 3. Keep that Chrome window OPEN.
echo 4. Run 'python server_web_scraper.py' in your terminal.
echo.

set CHROME_PATH=""

if exist "C:\Program Files\Google\Chrome\Application\chrome.exe" (
    set CHROME_PATH="C:\Program Files\Google\Chrome\Application\chrome.exe"
) else if exist "C:\Program Files (x86)\Google\Chrome\Application\chrome.exe" (
    set CHROME_PATH="C:\Program Files (x86)\Google\Chrome\Application\chrome.exe"
) else if exist "%LOCALAPPDATA%\Google\Chrome\Application\chrome.exe" (
    set CHROME_PATH="%LOCALAPPDATA%\Google\Chrome\Application\chrome.exe"
)

if %CHROME_PATH%=="" (
    echo ❌ Google Chrome was not found in default locations.
    pause
    exit /b
)

start "" %CHROME_PATH% --remote-debugging-port=9222 --user-data-dir="%LOCALAPPDATA%\Google\Chrome\RobotProfile" "https://gemini.google.com/app"
echo ✅ Chrome started with remote debugging!
