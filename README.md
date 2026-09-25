# 🌐 Gemini Web Chat Automation (Zero API Keys / Free Unlimited Mode)

This setup connects directly to your **logged-in Google Chrome browser** on `gemini.google.com`, bypassing the Developer API entirely so you **never hit any 429 quota limits**!

---

## 🚀 How to Run in 3 Easy Steps:

### 1. Launch Chrome with Remote Debugging
Double-click [`launch_chrome.bat`](file:///c:/Users/Jeremy/Desktop/SCHOOL%20ACTIVITIES/500%20level/Final_Year_Project/Auto_Waste_Bin/CODE/TEST_WEB_Scrap/launch_chrome.bat) or run in PowerShell:
```powershell
cd "c:\Users\Jeremy\Desktop\SCHOOL ACTIVITIES\500 level\Final_Year_Project\Auto_Waste_Bin\CODE\TEST_WEB_Scrap"
.\launch_chrome.bat
```
* A special Chrome window will open to `https://gemini.google.com/app`.
* **Sign in to your Google / Gemini account** in that window.
* **Leave that Chrome window open in the background!**

---

### 2. Start the Web Scraper Server
In your PowerShell terminal, run:
```powershell
python server_web_scraper.py
```
* It will connect to your opened Chrome window over port `9222`.
* It prints: `✅ Connected to Chrome! Title: Gemini`.

---

### 3. Upload & Run
* Open `http://localhost:5000` to see the live feed and real-time typed prompts.
* Connect to your robot at `http://192.168.4.1` and click **CAM Guided Auto**.
* As the robot scans, frames are uploaded directly into your Gemini web chat, and responses are extracted in real-time!
