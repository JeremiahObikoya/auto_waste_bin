"""
=============================================================================
AUTO WASTE BIN V2 — GEMINI WEB SCRAPER & BROWSER AUTOMATION SERVER
=============================================================================
This server connects to an existing, logged-in Google Chrome browser
running at 'http://localhost:9222' (launched via launch_chrome.bat).

It sends camera frames directly into your Gemini web chat (gemini.google.com),
bypassing all Developer API quotas, rate limits, and billing keys!
=============================================================================
"""

import os
import time
import json
import socket
import threading
from flask import Flask, request, jsonify, Response
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

app = Flask(__name__)

# Server configuration
HOST = "0.0.0.0"
PORT = 5000
DEBUG_DIR = "debug_frames"
if not os.path.exists(DEBUG_DIR):
    os.makedirs(DEBUG_DIR)

# System Prompts for each phase
PROMPTS = {
    0: (
        "You are the vision system of an autonomous waste bin robot. "
        "Analyse this camera frame and determine if a person has an EXTENDED ARM "
        "clearly HOLDING AN OBJECT (e.g. trash, bottle, paper) directed toward the camera. "
        "Respond ONLY with raw JSON. "
        "If detected: {\"detection\": \"EXTENDED_ARM_WITH_OBJECT\", \"bbox_center_x\": PIXEL} "
        "where PIXEL (0-320) is horizontal pixel of target centre. "
        "If not detected: {\"detection\": \"NONE\"}"
    ),
    1: (
        "You are the vision system of an autonomous waste bin robot in TRACKING mode. "
        "The robot is moving toward a person. Identify ONE of the following: "
        "STOP_PALM - person raises an open empty palm toward camera (stop gesture); "
        "PROXIMITY - person fills more than 65% of frame width (robot is very close); "
        "TRACKING - person is visible with arm extended holding object. "
        "Respond ONLY with raw JSON. "
        "Format: {\"detection\": \"STOP_PALM\"} "
        "or {\"detection\": \"PROXIMITY\"} "
        "or {\"detection\": \"TRACKING\", \"bbox_center_x\": PIXEL}"
    ),
    2: (
        "You are the vision system of an autonomous waste bin robot in drop-off mode. "
        "The robot is stationary waiting for user to deposit waste. "
        "Identify ONE of the following: "
        "EMPTY_ARM - person's arm is extended but empty (waste dropped); "
        "BACK_TURNED - person turned back and is walking away; "
        "WAITING - person still holding waste. "
        "Respond ONLY with raw JSON: "
        "{\"detection\": \"EMPTY_ARM\"} or {\"detection\": \"BACK_TURNED\"} or {\"detection\": \"WAITING\"}"
    )
}

# State variables
latest_frame = None
latest_result = {"detection": "NONE", "bbox_center_x": 160}
latest_latency = {"net_kb": 0, "gemini_s": 0.0, "total_s": 0.0}
latest_prompt = "Waiting for first frame..."
latest_raw_response = "Waiting for Gemini web output..."
latest_error = ""
stats = {"total_frames": 0, "detections": 0, "errors": 0}

driver = None
driver_lock = threading.Lock()

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def init_selenium_browser():
    """Connect to Google Chrome running with remote debugging on port 9222."""
    global driver, latest_error
    print("🔌 Connecting to Google Chrome on localhost:9222...")
    chrome_options = Options()
    chrome_options.add_experimental_option("debuggerAddress", "127.0.0.1:9222")
    try:
        driver = webdriver.Chrome(options=chrome_options)
        print(f"✅ Connected to Chrome! Title: {driver.title}")
        if "gemini.google.com" not in driver.current_url:
            print("🌐 Navigating to https://gemini.google.com/app ...")
            driver.get("https://gemini.google.com/app")
            time.sleep(3)
        latest_error = ""
        return True
    except Exception as e:
        err = f"Could not connect to Chrome on port 9222: {e}. Make sure launch_chrome.bat is running!"
        print(f"❌ {err}")
        latest_error = err
        return False

def query_gemini_web(image_path: str, prompt_text: str):
    """
    Automates uploading the image into the gemini.google.com chat,
    sending the prompt, and extracting the response text.
    """
    global driver, latest_error
    if driver is None:
        if not init_selenium_browser():
            return {"detection": "NONE"}, 0.0, "Error: Browser not connected. Run launch_chrome.bat", latest_error

    with driver_lock:
        t0 = time.time()
        try:
            # 1. Locate file input for image upload
            file_inputs = driver.find_elements(By.CSS_SELECTOR, "input[type='file']")
            if not file_inputs:
                # Try finding and clicking the 'Add file' / '+' button if hidden
                try:
                    plus_btn = driver.find_element(By.CSS_SELECTOR, "button[aria-label*='Upload'], button[aria-label*='Add'], button[mattooltip*='Upload']")
                    plus_btn.click()
                    time.sleep(0.5)
                    file_inputs = driver.find_elements(By.CSS_SELECTOR, "input[type='file']")
                except Exception:
                    pass

            if file_inputs:
                file_input = file_inputs[0]
                file_input.send_keys(os.path.abspath(image_path))
                time.sleep(0.8)  # Wait for upload thumbnail to attach
            else:
                print("⚠ Warning: Image upload input not found on page.")

            # 2. Locate the prompt input box
            text_boxes = driver.find_elements(By.CSS_SELECTOR, "div[role='textbox'], rich-textarea p, textarea")
            if not text_boxes:
                raise Exception("Gemini prompt input box not found on page.")
            
            box = text_boxes[0]
            try:
                driver.execute_script("arguments[0].focus();", box)
            except Exception:
                pass
            box.click()
            box.send_keys(prompt_text)
            time.sleep(0.4)
            
            # Click Send button or send Enter key
            send_btns = driver.find_elements(By.CSS_SELECTOR, "button[aria-label*='Send'], button[aria-label*='send'], button.send-button")
            sent = False
            if send_btns:
                for b in send_btns:
                    if b.is_displayed() and b.is_enabled():
                        b.click()
                        sent = True
                        break
            if not sent:
                box.send_keys(Keys.ENTER)

            # 3. Wait for the generation to finish
            # Track response elements until the streaming indicator disappears
            time.sleep(1.2)
            max_wait = 25
            start_wait = time.time()
            raw_text = ""

            while time.time() - start_wait < max_wait:
                try:
                    responses = driver.find_elements(By.CSS_SELECTOR, "message-content, .model-response-text, .response-container, .markdown")
                    if responses:
                        last_resp = responses[-1]
                        txt = last_resp.text.strip()
                        if txt:
                            raw_text = txt

                        # Check if send/stop button indicates generation is finished
                        send_btns = driver.find_elements(By.CSS_SELECTOR, "button[aria-label*='Send'], button[aria-label*='send'], button[aria-label*='stop']")
                        is_generating = any("stop" in (b.get_attribute("aria-label") or "").lower() for b in send_btns)
                        
                        # If send button is back or text contains valid complete JSON structure
                        if not is_generating and ("{" in raw_text and "}" in raw_text):
                            break
                except Exception:
                    # Catch StaleElementReferenceException while DOM is re-rendering
                    time.sleep(0.3)
                    continue

                time.sleep(0.4)

            gemini_dur = time.time() - t0
            
            # Clean and parse JSON
            cleaned = raw_text.strip()
            if "```json" in cleaned:
                cleaned = cleaned.split("```json")[1].split("```")[0]
            elif "```" in cleaned:
                cleaned = cleaned.split("```")[1].split("```")[0]
            cleaned = cleaned.strip()

            try:
                res_json = json.loads(cleaned)
                if "detection" in res_json:
                    return res_json, gemini_dur, raw_text, ""
            except Exception:
                pass

            if "EXTENDED_ARM_WITH_OBJECT" in raw_text:
                return {"detection": "EXTENDED_ARM_WITH_OBJECT", "bbox_center_x": 160}, gemini_dur, raw_text, ""
            elif "STOP_PALM" in raw_text:
                return {"detection": "STOP_PALM"}, gemini_dur, raw_text, ""
            elif "PROXIMITY" in raw_text:
                return {"detection": "PROXIMITY"}, gemini_dur, raw_text, ""
            elif "EMPTY_ARM" in raw_text:
                return {"detection": "EMPTY_ARM"}, gemini_dur, raw_text, ""
            elif "BACK_TURNED" in raw_text:
                return {"detection": "BACK_TURNED"}, gemini_dur, raw_text, ""
            elif "TRACKING" in raw_text:
                return {"detection": "TRACKING", "bbox_center_x": 160}, gemini_dur, raw_text, ""
            elif "NONE" in raw_text:
                return {"detection": "NONE"}, gemini_dur, raw_text, ""
            else:
                return {"detection": "INVALID"}, gemini_dur, raw_text, "Incomplete or unrecognized response"

        except Exception as e:
            latest_error = str(e)
            print(f"❌ Web Scraping Error: {e}")
            return {"detection": "NONE"}, time.time() - t0, f"Error: {e}", str(e)

@app.route('/process_frame', methods=['POST'])
def process_frame():
    global latest_frame, latest_result, latest_latency, latest_prompt, latest_raw_response, stats
    t_start = time.time()
    
    image_bytes = request.get_data()
    if not image_bytes or len(image_bytes) < 100:
        return jsonify({"detection": "NONE", "error": "Invalid frame"}), 400
    
    state_code = request.args.get('state', default=0, type=int)
    is_preview = request.args.get('preview', default=0, type=int)
    
    latest_frame = image_bytes
    frame_kb = len(image_bytes) / 1024.0
    stats["total_frames"] += 1
    
    # If preview frame, do not send to Gemini chat
    if is_preview:
        return jsonify({"status": "preview_ok", "detection": "NONE"})
        
    # Assign system prompt for current vision state
    prompt = PROMPTS.get(state_code, PROMPTS[0])
    latest_prompt = prompt
    
    # Save frame with unique timestamp and frame counter to prevent Gemini duplicate file warnings
    unique_name = f"frame_{int(time.time()*1000)}_{stats['total_frames']}.jpg"
    temp_path = os.path.abspath(os.path.join(DEBUG_DIR, unique_name))
    with open(temp_path, "wb") as f:
        f.write(image_bytes)
            
    # Send frame to logged-in Gemini web browser
    result, gemini_dur, raw_resp, err_str = query_gemini_web(temp_path, prompt)
    total_dur = time.time() - t_start
    
    # Clean up temporary frame after query if debug saving is off
    try:
        if os.path.exists(temp_path):
            os.remove(temp_path)
    except Exception:
        pass
    
    latest_result = result
    latest_raw_response = raw_resp
    latest_latency = {
        "net_kb": round(frame_kb, 1),
        "gemini_s": round(gemini_dur, 2),
        "total_s": round(total_dur, 2)
    }
    
    det = result.get("detection", "NONE")
    bx = result.get("bbox_center_x", 160)
    
    if det != "NONE":
        stats["detections"] += 1
        print(f"🎯 [{time.strftime('%H:%M:%S')}] State={state_code} | Frame: {frame_kb:.1f} KB | Gemini Web: {gemini_dur:.2f}s | Decision: {det} (x={bx}) | Total: {total_dur:.2f}s")
    elif err_str:
        stats["errors"] += 1
    else:
        print(f"🔍 [{time.strftime('%H:%M:%S')}] State={state_code} | Frame: {frame_kb:.1f} KB | Gemini Web: {gemini_dur:.2f}s | Result: NONE | Total: {total_dur:.2f}s")
        
    return jsonify(result)

@app.route('/latest_frame.jpg')
def get_latest_frame():
    if latest_frame is not None:
        return Response(latest_frame, mimetype='image/jpeg', headers={'Cache-Control': 'no-cache, no-store, must-revalidate'})
    return Response(b'', status=404)

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
            time.sleep(0.15)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Auto Waste Bin V2 — Gemini Web Chat Automation</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body {{ font-family: 'Segoe UI', system-ui, sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; }}
            .container {{ max-width: 980px; margin: 0 auto; display: flex; flex-direction: column; gap: 16px; }}
            h1 {{ color: #38bdf8; font-size: 22px; margin: 0; }}
            .card {{ background: #1e293b; border-radius: 12px; padding: 18px; border: 1px solid #334155; }}
            .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 12px; }}
            .stat-box {{ background: #0f172a; padding: 12px; border-radius: 8px; border: 1px solid #334155; text-align: center; }}
            .stat-label {{ font-size: 11px; color: #94a3b8; text-transform: uppercase; letter-spacing: 0.05em; }}
            .stat-val {{ font-size: 20px; font-weight: bold; color: #38bdf8; margin-top: 4px; }}
            .badge {{ display: inline-block; padding: 4px 10px; border-radius: 20px; font-weight: bold; font-size: 12px; }}
            .badge-green {{ background: rgba(16, 185, 129, 0.2); color: #34d399; border: 1px solid #10b981; }}
            .feed-img {{ width: 100%; max-width: 440px; border-radius: 8px; border: 2px solid #334155; display: block; margin: 0 auto; }}
            .ip-box {{ background: #0284c7; color: white; padding: 6px 12px; border-radius: 6px; font-family: monospace; font-size: 13px; }}
            .two-col {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }}
            .terminal {{ background: #090d16; border: 1px solid #334155; border-radius: 8px; padding: 12px; font-family: monospace; font-size: 12px; color: #7ee787; height: 160px; overflow-y: auto; white-space: pre-wrap; word-break: break-all; }}
            .err-banner {{ background: rgba(239, 68, 68, 0.15); border: 1px solid #ef4444; color: #fca5a5; padding: 12px 16px; border-radius: 8px; font-size: 13px; display: none; }}
            @media(max-width: 768px) {{ .two-col {{ grid-template-columns: 1fr; }} }}
        </style>
        <script>
            function refreshFeed() {{
                const img = document.getElementById('camera-img');
                img.src = '/latest_frame.jpg?t=' + new Date().getTime();
            }}
            setInterval(refreshFeed, 300);

            setInterval(() => {{
                fetch('/api/status').then(r => r.json()).then(d => {{
                    document.getElementById('det-val').innerText = d.result.detection || 'NONE';
                    document.getElementById('bbox-val').innerText = d.result.bbox_center_x !== undefined ? d.result.bbox_center_x : '--';
                    document.getElementById('gem-lat').innerText = d.latency.gemini_s + ' s';
                    document.getElementById('tot-lat').innerText = d.latency.total_s + ' s';
                    document.getElementById('tot-frames').innerText = d.stats.total_frames;
                    document.getElementById('tot-dets').innerText = d.stats.detections;
                    document.getElementById('prompt-view').innerText = d.prompt;
                    document.getElementById('resp-view').innerText = d.raw_response;

                    let errEl = document.getElementById('err-banner');
                    if (d.error && d.error.length > 0) {{
                        errEl.style.display = 'block';
                        errEl.innerHTML = '<b>⚠️ Automation Status:</b> ' + d.error;
                    }} else {{
                        errEl.style.display = 'none';
                    }}
                }});
            }}, 600);
        </script>
    </head>
    <body>
        <div class="container">
            <div class="card" style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:10px;">
                <div>
                    <h1>🤖 Auto Waste Bin V2 — Gemini Web Chat Automation</h1>
                    <span class="badge badge-green" style="margin-top:6px;">● BROWSER AUTOMATION ONLINE</span>
                    <span style="color:#94a3b8; font-size:13px; margin-left:8px;">Target: <b>gemini.google.com/app</b></span>
                </div>
                <div>
                    <span style="font-size:13px; color:#94a3b8; margin-right:6px;">ESP32-CAM Target:</span>
                    <span class="ip-box">http://{get_local_ip()}:{PORT}/process_frame</span>
                </div>
            </div>

            <div id="err-banner" class="err-banner"></div>

            <div class="grid">
                <div class="stat-box">
                    <div class="stat-label">Latest Decision</div>
                    <div class="stat-val" id="det-val" style="color:#34d399; font-size:16px;">NONE</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Target Center X</div>
                    <div class="stat-val" id="bbox-val">--</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Gemini Web Latency</div>
                    <div class="stat-val" id="gem-lat">0.0 s</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Total Roundtrip</div>
                    <div class="stat-val" id="tot-lat">0.0 s</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Total Frames</div>
                    <div class="stat-val" id="tot-frames">0</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Detections</div>
                    <div class="stat-val" id="tot-dets">0</div>
                </div>
            </div>

            <div class="two-col">
                <div class="card" style="text-align:center;">
                    <h3 style="margin-top:0; color:#94a3b8; font-size:13px; text-transform:uppercase;">Live Feed from ESP32-CAM</h3>
                    <img id="camera-img" class="feed-img" src="/latest_frame.jpg" alt="Camera Feed">
                </div>

                <div class="card" style="display:flex; flex-direction:column; gap:10px;">
                    <div>
                        <h4 style="margin:0 0 6px 0; color:#38bdf8; font-size:12px; text-transform:uppercase;">📤 Prompt Typed into Gemini Web:</h4>
                        <div class="terminal" id="prompt-view" style="height:70px; color:#93c5fd;">Waiting for frame...</div>
                    </div>
                    <div>
                        <h4 style="margin:0 0 6px 0; color:#34d399; font-size:12px; text-transform:uppercase;">📥 Raw Gemini Web Response:</h4>
                        <div class="terminal" id="resp-view" style="height:90px;">Waiting for Gemini output...</div>
                    </div>
                </div>
            </div>
        </div>
    </body>
    </html>
    """
    return html

@app.route('/api/status')
def api_status():
    return jsonify({
        "result": latest_result,
        "latency": latest_latency,
        "stats": stats,
        "prompt": latest_prompt,
        "raw_response": latest_raw_response,
        "error": latest_error
    })

if __name__ == '__main__':
    local_ip = get_local_ip()
    print("=" * 65)
    print(" 🚀 GEMINI WEB CHAT AUTOMATION SERVER STARTED")
    print("=" * 65)
    print(f" 💻 Laptop Local IP Address : {local_ip}")
    print(f" 📡 ESP32-CAM Target URL    : http://{local_ip}:{PORT}/process_frame")
    print(f" 🌐 Web Visual Dashboard    : http://localhost:{PORT}")
    print("=" * 65)
    print(" Initializing Chrome connection...")
    init_selenium_browser()
    print(" Waiting for camera frames from ESP32-CAM...\n")
    app.run(host=HOST, port=PORT, debug=False, threaded=True)
