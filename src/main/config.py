"""
Configuration file for the Laptop Middleman Server
Auto Waste Bin V2 - Python Vision Backend
"""

# Gemini API Key (replace with your key or set GEMINI_API_KEY environment variable)
GEMINI_API_KEY = "AQ.Ab8RN6LzI5VSOmQZm6PkMnq9DkkhQELDZYlHxQyg7IM1TBSk_Q"

# Gemini Model Identifier
GEMINI_MODEL = "gemini-3.6-flash"

# Server Host & Port
# '0.0.0.0' allows connections from any device on your local Wi-Fi (e.g., ESP32-CAM)
HOST = "0.0.0.0"
PORT = 5000

# Save received debug frames to disk? (True/False)
SAVE_DEBUG_FRAMES = True
DEBUG_DIR = "debug_frames"
