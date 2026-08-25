"""
=============================================================================
AUTO WASTE BIN V2 — LAPTOP MIDDLEMAN VISION SERVER
=============================================================================
Receives raw JPEG frames from ESP32-CAM via fast local HTTP,
queries Google Gemini Vision API, measures exact latency, and returns
rapid JSON control decisions back to the ESP32-CAM.
=============================================================================
"""

import os
import time
import json
import socket
import base64
import requests
from flask import Flask, request, jsonify, Response
from config import GEMINI_API_KEY, GEMINI_MODEL, HOST, PORT, SAVE_DEBUG_FRAMES, DEBUG_DIR

app = Flask(__name__)

if SAVE_DEBUG_FRAMES and not os.path.exists(DEBUG_DIR):
    os.makedirs(DEBUG_DIR)

# System Prompts for each vision phase
PROMPTS = {
    0: (  # SCANNING
        "You are the vision system of an autonomous waste bin robot. "
        "Analyse this camera frame and determine if a person has an EXTENDED ARM "
        "clearly HOLDING AN OBJECT (e.g. trash, bottle, paper) directed toward the camera. "
        "Respond ONLY with raw JSON. "
        "If detected: {\"detection\": \"EXTENDED_ARM_WITH_OBJECT\", \"bbox_center_x\": PIXEL} "
        "where PIXEL (0-320) is horizontal pixel of target centre. "
        "If not detected: {\"detection\": \"NONE\"}"
    ),
    1: (  # TRACKING
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
    2: (  # INTERACTION / DROP-OFF
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

# State variables for web dashboard & inspection
latest_frame = None
latest_result = {"detection": "NONE", "bbox_center_x": 160}
latest_latency = {"net_kb": 0, "gemini_s": 0.0, "total_s": 0.0}
latest_prompt = "No query sent yet."
latest_raw_response = "Waiting for first frame..."
latest_error = ""
stats = {"total_frames": 0, "detections": 0, "errors": 0}

def get_local_ip():
    """Find the laptop's Wi-Fi IP address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def query_gemini_rest(image_bytes: bytes, prompt_text: str):
    """
    POST base64 image + prompt to Gemini REST API.
    Returns (decision_dict, gemini_duration_seconds, raw_text_response, error_str).
    """
    global latest_error
    b64_image = base64.b64encode(image_bytes).decode('utf-8')
    url = f"https://generativelanguage.googleapis.com/v1beta/models/{GEMINI_MODEL}:generateContent?key={GEMINI_API_KEY}"
    
    payload = {
        "contents": [
            {
                "parts": [
                    {"text": prompt_text},
                    {
                        "inline_data": {
                            "mime_type": "image/jpeg",
                            "data": b64_image
                        }
                    }
                ]
            }
        ]
    }
    
    t0 = time.time()
    try:
        response = requests.post(url, json=payload, headers={"Content-Type": "application/json"}, timeout=12)
        gemini_dur = time.time() - t0
        
        if response.status_code == 200:
            latest_error = ""
            data = response.json()
            raw_text = data["candidates"][0]["content"]["parts"][0]["text"]
            
            # Clean markdown wrappers
            cleaned = raw_text.strip()
            if cleaned.startswith("```json"):
                cleaned = cleaned[7:]
            if cleaned.startswith("```"):
                cleaned = cleaned[3:]
            if cleaned.endswith("```"):
                cleaned = cleaned[:-3]
            cleaned = cleaned.strip()
            
            try:
                res_json = json.loads(cleaned)
                return res_json, gemini_dur, raw_text, ""
            except Exception:
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
                else:
                    return {"detection": "NONE"}, gemini_dur, raw_text, ""
        else:
            err_msg = response.text
            latest_error = f"HTTP {response.status_code}: {err_msg}"
            if response.status_code == 429:
                print("\n" + "!" * 70)
                print(" 🚨 GEMINI 429 QUOTA EXHAUSTED: You hit the free preview limit (20 req/day).")
                print(" 💡 SOLUTION: Enable Pay-as-you-go or link your API Key to a Google Cloud Project with billing.")
                print("!" * 70 + "\n")
            else:
                print(f"❌ Gemini API Error [{response.status_code}]: {err_msg}")
            return {"detection": "NONE"}, gemini_dur, f"ERROR {response.status_code}: {err_msg}", err_msg
    except Exception as e:
        latest_error = str(e)
        print(f"❌ Network Error to Gemini: {e}")
        return {"detection": "NONE"}, time.time() - t0, f"NETWORK ERROR: {e}", str(e)

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
    
    # If this is just a preview stream (e.g. manual mode), don't query Gemini
    if is_preview:
        return jsonify({"status": "preview_ok", "detection": "NONE"})
        
    prompt = PROMPTS.get(state_code, PROMPTS[0])
    latest_prompt = prompt
    
    # Save debug snapshot
    if SAVE_DEBUG_FRAMES:
        fname = os.path.join(DEBUG_DIR, f"frame_{int(time.time()*1000)}_s{state_code}.jpg")
        with open(fname, "wb") as f:
            f.write(image_bytes)
            
    # Query Gemini
    result, gemini_dur, raw_resp, err_str = query_gemini_rest(image_bytes, prompt)
    total_dur = time.time() - t_start
    
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
        print(f"🎯 [{time.strftime('%H:%M:%S')}] State={state_code} | Frame: {frame_kb:.1f} KB | Gemini: {gemini_dur:.2f}s | Decision: {det} (x={bx}) | Total: {total_dur:.2f}s")
    elif err_str:
        stats["errors"] += 1
    else:
        print(f"🔍 [{time.strftime('%H:%M:%S')}] State={state_code} | Frame: {frame_kb:.1f} KB | Gemini: {gemini_dur:.2f}s | Result: NONE | Total: {total_dur:.2f}s")
        
    return jsonify(result)

@app.route('/latest_frame.jpg')
def get_latest_frame():
    """Return the most recent JPEG frame directly."""
    if latest_frame is not None:
        return Response(latest_frame, mimetype='image/jpeg', headers={'Cache-Control': 'no-cache, no-store, must-revalidate'})
    return Response(b'', status=404)

@app.route('/video_feed')
def video_feed():
    """Stream the latest captured frame."""
    def gen():
        while True:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
            time.sleep(0.15)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    """Live Visual Dashboard & Gemini Inspector."""
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Auto Waste Bin V2 — Vision Middleman Dashboard</title>
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
            let lastUpdate = 0;
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
                        errEl.innerHTML = '<b>⚠️ Gemini Error:</b> ' + d.error;
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
                    <h1>🤖 Auto Waste Bin V2 — Vision Middleman</h1>
                    <span class="badge badge-green" style="margin-top:6px;">● ONLINE</span>
                    <span style="color:#94a3b8; font-size:13px; margin-left:8px;">Model: <b>{GEMINI_MODEL}</b></span>
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
                    <div class="stat-label">Gemini Latency</div>
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
                        <h4 style="margin:0 0 6px 0; color:#38bdf8; font-size:12px; text-transform:uppercase;">📤 Prompt Sent to Gemini:</h4>
                        <div class="terminal" id="prompt-view" style="height:70px; color:#93c5fd;">Waiting for frame...</div>
                    </div>
                    <div>
                        <h4 style="margin:0 0 6px 0; color:#34d399; font-size:12px; text-transform:uppercase;">📥 Raw Gemini Model Response:</h4>
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
    print(" 🚀 AUTO WASTE BIN V2 — MIDDLEMAN VISION SERVER STARTED")
    print("=" * 65)
    print(f" 💻 Laptop Local IP Address : {local_ip}")
    print(f" 📡 ESP32-CAM Target URL    : http://{local_ip}:{PORT}/process_frame")
    print(f" 🌐 Web Visual Dashboard    : http://localhost:{PORT}")
    print(f" 🤖 Gemini Model Configured : {GEMINI_MODEL}")
    print("=" * 65)
    print(" Waiting for camera frames from ESP32-CAM...\n")
    app.run(host=HOST, port=PORT, debug=False, threaded=True)
