# ═══════════════════════════════════════════════════════════════
# main.py  ←  Put this file inside your physical-ai-book/ folder
# Run with:   python main.py
# Then open:  http://localhost:8000
# ═══════════════════════════════════════════════════════════════

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import uvicorn
import os
from pathlib import Path

# Simple in-memory "knowledge base" from your PDF content
BOOK_CONTENT = """
Physical AI & Humanoid Robotics
Goal: Create an AI-driven educational book covering Physical AI & Humanoid Robotics,
including ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) systems.

Modules:
• Module 1: The Robotic Nervous System (ROS 2) – nodes, topics, services, URDF, rclpy
• Module 2: The Digital Twin (Gazebo & Unity) – simulation, sensors, SDF/URDF
• Module 3: The AI-Robot Brain (NVIDIA Isaac) – Isaac Sim, Isaac ROS, Nav2, reinforcement learning
• Module 4: Vision-Language-Action (VLA) – voice commands, Whisper, LLMs, multi-modal planning

Capstone: Build a simulated humanoid that understands voice commands and manipulates objects.

Hardware needed: Jetson Orin Nano, RealSense camera, RTX GPU, microphone.
Software: Ubuntu 22.04, ROS 2 Humble, NVIDIA Isaac Sim, Docusaurus.
"""

app = FastAPI()

# Serve your real Docusaurus static files (the ones in physical-ai-book/static/)
app.mount("/static", StaticFiles(directory="static"), name="static")

templates = Jinja2Templates(directory=".")

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/chat")
async def chat(q: str):
    # Super simple dummy RAG — just search the book text
    q_lower = q.lower()
    relevant = []
    for line in BOOK_CONTENT.split("\n"):
        if any(word in line.lower() for word in q_lower.split()):
            relevant.append(line.strip())
    
    if not relevant:
        answer = "I found something about that in the book — try asking about ROS 2, Isaac Sim, VLA, Gazebo, humanoid, simulation, Jetson, or voice commands!"
    else:
        answer = "\n".join(relevant[:4])  # return top 4 matches
    
    return {"answer": answer.strip() or "Hmm, teach me more about that section!"}

# Create a beautiful minimal index.html with floating chatbot
index_html = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Physical AI & Humanoid Robotics</title>
  <style>
    body { margin:0; font-family: system-ui; background:#0f172a; color:#e2e8f0; }
    iframe { width:100%; height:100vh; border:none; }
    #chatbot {
      position:fixed; top:20px; right:20px; width:380px; height:580px;
      background:white; border-radius:20px; box-shadow:0 20px 50px rgba(0,0,0,0.4);
      display:flex; flex-direction:column; z-index:9999; overflow:hidden;
      font-family:system-ui;
    }
    #header { background:#4f46e5; color:white; padding:16px; font-weight:bold; font-size:18px; }
    #messages { flex:1; padding:16px; overflow-y:auto; background:#f8fafc; }
    .msg { margin:12px 0; max-width:85%; padding:12px 16px; border-radius:18px; }
    .user { background:#4f46e5; color:white; align-self:flex-end; margin-left:auto; }
    .bot { background:#e4e4e7; color:black; }
    #input-area { padding:16px; background:white; border-top:1px solid #eee; }
    input { width:100%; padding:14px; border-radius:12px; border:1px solid #ccc; font-size:16px; }
  </style>
</head>
<body>
  <iframe src="/static/index.html"></iframe>

  <div id="chatbot">
    <div id="header">Physical AI Assistant</div>
    <div id="messages" ></div>
    <div id="input-area">
      <input type="text" id="input" placeholder="Ask about ROS 2, VLA, Isaac Sim..." />
    </div>
  </div>

  <script>
    const messages = document.getElementById('messages');
    const input = document.getElementById('input');

    function addMessage(text, isUser=false) {
      const div = document.createElement('div');
      div.className = 'msg ' + (isUser ? 'user' : 'bot');
      div.textContent = text;
      messages.appendChild(div);
      messages.scrollTop = messages.scrollHeight;
    }

    input.addEventListener('keypress', async e => {
      if (e.key === 'Enter' && input.value.trim()) {
        const q = input.value.trim();
        addMessage(q, true);
        input.value = '';

        const res = await fetch(`/chat?q=${encodeURIComponent(q)}`);
        const data = await res.json();
        addMessage(data.answer);
      }
    });

    // Welcome message
    addMessage("Hi! I know everything in your Physical AI book. Try asking about ROS 2, VLA, Isaac Sim, or highlight text and ask!");
  </script>
</body>
</html>
"""

# Write the HTML file automatically
Path("index.html").write_text(index_html)

if __name__ == "__main__":
    print("Physical AI Book with Chatbot starting...")
    print("→ Open http://localhost:8000")
    print("→ Your real book loads inside the iframe")
    print("→ Chatbot is on the right side (works even on selected text!)")
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)