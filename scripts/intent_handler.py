#!/usr/bin/env python3

import subprocess
import time
import openai
import os

# === CONFIG ===
OPENAI_API_KEY = "sk-abcdqrstefgh5678abcdqrstefgh5678abcdqrst-"
AI_COMMAND = ["rosrun", "smart_shopping_bot", "intent_ai.py"]
OFFLINE_COMMAND = ["rosrun", "smart_shopping_bot", "intent_offline.py"]
CHECK_INTERVAL = 2  # seconds

# === FUNCTIONS ===
def test_openai_connection():
    try:
        openai.api_key = OPENAI_API_KEY
        openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": "Hello"}],
            timeout=5
        )
        return True
    except Exception as e:
        print(f"❌ OpenAI not available: {e}")
        return False

def run_node(command):
    print(f"🚀 Launching: {' '.join(command)}")
    return subprocess.Popen(command)

def main():
    print("🔍 Checking OpenAI API availability...")
    use_ai = test_openai_connection()

    selected_command = AI_COMMAND if use_ai else OFFLINE_COMMAND
    process = run_node(selected_command)

    try:
        while True:
            if process.poll() is not None:
                print("⚠️ Intent handler crashed. Restarting fallback...")
                process.terminate()
                time.sleep(1)
                process = run_node(OFFLINE_COMMAND)
            time.sleep(CHECK_INTERVAL)
    except KeyboardInterrupt:
        print("🛑 Shutdown requested. Terminating process...")
        process.terminate()

if __name__ == "__main__":
    main()
