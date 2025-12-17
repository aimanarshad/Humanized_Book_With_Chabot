import React, { useState, useRef, useEffect } from "react";

export default function Chatbot() {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const messagesEndRef = useRef(null);

  const BACKEND_API_URL = "http://localhost:8000";

  // Docusaurus-inspired light green theme colors
  const primaryColor = "#25c2a0";       // Main teal-green
  const primaryDark = "#1fa588";
  const primaryLight = "#5cd6b8";
  const primaryLighter = "#92e0cb";
  const primaryLightest = "#c8ede3";

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Static fallback responses (in case API is down or unreachable)
  const staticResponses = {
    "what is physical ai": `
Physical AI (also known as embodied AI) is an emerging field that integrates advanced AI with physical robotic systems, enabling machines to perceive, understand, interact with, and act intelligently in the real world.

Key characteristics:
- Combines large AI foundation models (like vision-language-action models) with robotics hardware.
- Allows robots to adapt to new environments, plan multi-step tasks, recover from errors, and operate under uncertainty.
- Especially impactful for **humanoid robots**, which are designed to work safely alongside humans in everyday spaces.

Examples of leading efforts:
- NVIDIA's Project GR00T: A foundation model for humanoid robot reasoning and skills.
- Companies like Tesla (Optimus), Figure AI, Boston Dynamics, and Agility Robotics are building commercial humanoid robots powered by Physical AI.

This convergence is expected to drive the next wave of industrial automation, productivity gains, and transformative applications in manufacturing, logistics, healthcare, and beyond.

Sources: NVIDIA Glossary, Deloitte Insights, World Economic Forum (2025 reports)
    `.trim(),

    // You can add more static fallbacks here if needed
  };

  async function sendMessage() {
    if (!input.trim() || isLoading) return;

    const normalizedInput = input.trim().toLowerCase();

    // Check for static fallback first
    if (staticResponses[normalizedInput]) {
      const userMessage = { sender: "user", text: input };
      setMessages((prev) => [...prev, userMessage]);

      const botMessage = {
        sender: "bot",
        text: staticResponses[normalizedInput],
        sources: [],
      };
      setMessages((prev) => [...prev, botMessage]);
      setInput("");
      return;
    }

    // Otherwise, try the API
    const userMessage = { sender: "user", text: input };
    setMessages((prev) => [...prev, userMessage]);
    setInput("");
    setIsLoading(true);

    try {
      const response = await fetch(`${BACKEND_API_URL}/v1/query`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-API-Key": "4c3eecd9308b619594f73ee4c852a02330c68a37063fdf74", // Hardcoded for local dev
        },
        body: JSON.stringify({
          question: input,
          conversation_id: conversationId || undefined,
        }),
      });

      if (!response.ok) {
        throw new Error(`API Error: ${response.status}`);
      }

      const data = await response.json();

      if (!conversationId && data.conversation_id) {
        setConversationId(data.conversation_id);
      }

      const botReply = data.answer || "Sorry, I couldn't generate a response.";
      const sources = data.sources || [];

      let botResponse = botReply;
      if (sources.length > 0) {
        botResponse += `\n\nSources: ${sources.slice(0, 3).join(", ")}`;
      }

      const botMessage = {
        sender: "bot",
        text: botResponse,
        sources,
      };
      setMessages((prev) => [...prev, botMessage]);
    } catch (err) {
      console.error(err);
      const errorMessage = {
        sender: "bot",
        text: "⚠️ The backend API is currently unavailable.\n\nHere's a quick answer based on known course information:\n\n" +
          (staticResponses[normalizedInput] || "Sorry, I don't have a cached answer for that question right now. Please try again later when the API is back online."),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  }

  const handleKeyPress = (e) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div
      style={{
        minHeight: "100vh",
        background: `linear-gradient(135deg, #e8f7f3 0%, #f0faf7 100%)`,
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        padding: "20px",
        fontFamily: "-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif",
      }}
    >
      <div
        style={{
          width: "100%",
          maxWidth: "900px",
          height: "90vh",
          background: "rgba(255, 255, 255, 0.95)",
          borderRadius: "24px",
          boxShadow: "0 20px 60px rgba(0, 0, 0, 0.12)",
          display: "flex",
          flexDirection: "column",
          overflow: "hidden",
          backdropFilter: "blur(20px)",
          border: `1px solid ${primaryLighter}`,
        }}
      >
        {/* Header - Docusaurus-style green */}
        <div
          style={{
            background: `linear-gradient(135deg, ${primaryColor} 0%, ${primaryDark} 100%)`,
            color: "white",
            padding: "32px 40px",
            textAlign: "center",
            position: "relative",
          }}
        >
          <h1 style={{ fontSize: "2.2em", fontWeight: "700", margin: 0 }}>
            AI Book Assistant
          </h1>
          <p style={{ opacity: 0.9, margin: "12px 0 0", fontSize: "1.1em" }}>
            Ask me anything about the Physical AI & Humanoid Robotics course content
          </p>
          <div
            style={{
              position: "absolute",
              bottom: "12px",
              left: "50%",
              transform: "translateX(-50%)",
              width: "80px",
              height: "5px",
              background: "rgba(255,255,255,0.4)",
              borderRadius: "3px",
            }}
          />
        </div>

        {/* Messages Area */}
        <div
          style={{
            flex: 1,
            overflowY: "auto",
            padding: "40px 40px 20px",
            background: "linear-gradient(to bottom, #f0faf7, #ffffff)",
            display: "flex",
            flexDirection: "column",
            gap: "24px",
          }}
        >
          {messages.length === 0 && (
            <div
              style={{
                textAlign: "center",
                color: "#64748b",
                marginTop: "80px",
                lineHeight: "1.8",
              }}
            >
              <div style={{ fontSize: "5em", marginBottom: "24px" }}>📚</div>
              <h3 style={{ fontSize: "1.8em", fontWeight: "600", margin: "0 0 12px" }}>
                Welcome to Your AI Assistant
              </h3>
              <p style={{ fontSize: "1.2em", opacity: 0.8, maxWidth: "600px", margin: "0 auto" }}>
                I'm here to help you explore the course material, answer questions, explain concepts, and guide you through the content.
              </p>
            </div>
          )}

          {messages.map((msg, i) => (
            <div
              key={i}
              style={{
                alignSelf: msg.sender === "user" ? "flex-end" : "flex-start",
                maxWidth: "80%",
              }}
            >
              <div
                style={{
                  padding: "20px 28px",
                  borderRadius: "24px",
                  background:
                    msg.sender === "user"
                      ? `linear-gradient(135deg, ${primaryColor}, ${primaryDark})`
                      : "#ffffff",
                  color: msg.sender === "user" ? "white" : "#1e293b",
                  boxShadow: "0 10px 30px rgba(0, 0, 0, 0.12)",
                  borderBottomRightRadius: msg.sender === "user" ? "6px" : "24px",
                  borderBottomLeftRadius: msg.sender === "bot" ? "6px" : "24px",
                  position: "relative",
                  fontSize: "16.5px",
                  lineHeight: "1.7",
                  whiteSpace: "pre-wrap",
                }}
              >
                {msg.text}
              </div>
            </div>
          ))}

          {isLoading && (
            <div style={{ alignSelf: "flex-start", maxWidth: "80%" }}>
              <div
                style={{
                  padding: "20px 28px",
                  background: "#ffffff",
                  borderRadius: "24px",
                  borderBottomLeftRadius: "6px",
                  boxShadow: "0 10px 30px rgba(0, 0, 0, 0.12)",
                  display: "inline-block",
                }}
              >
                <div style={{ display: "flex", gap: "10px" }}>
                  <div className="typing-dot" />
                  <div className="typing-dot" />
                  <div className="typing-dot" />
                </div>
              </div>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        {/* Input Area */}
        <div
          style={{
            padding: "30px 40px 40px",
            background: "rgba(255, 255, 255, 0.95)",
            borderTop: `1px solid ${primaryLighter}`,
          }}
        >
          <div
            style={{
              display: "flex",
              background: "white",
              borderRadius: "32px",
              boxShadow: "0 12px 40px rgba(0, 0, 0, 0.12)",
              border: `1px solid ${primaryLighter}`,
              overflow: "hidden",
              transition: "all 0.3s ease",
            }}
          >
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={handleKeyPress}
              placeholder="Type your question here..."
              disabled={isLoading}
              style={{
                flex: 1,
                border: "none",
                outline: "none",
                padding: "22px 30px",
                fontSize: "17px",
                background: "transparent",
                color: "#1e293b",
              }}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !input.trim()}
              style={{
                padding: "0 40px",
                background: `linear-gradient(135deg, ${primaryColor}, ${primaryDark})`,
                color: "white",
                border: "none",
                cursor: isLoading || !input.trim() ? "not-allowed" : "pointer",
                fontWeight: "600",
                fontSize: "17px",
                opacity: isLoading || !input.trim() ? 0.6 : 1,
                transition: "all 0.3s ease",
              }}
            >
              {isLoading ? "Sending..." : "Send"}
            </button>
          </div>
        </div>
      </div>

      {/* Global Typing Animation */}
      <style jsx global>{`
        @keyframes typing {
          0%,
          100% {
            transform: translateY(0);
            opacity: 0.4;
          }
          50% {
            transform: translateY(-10px);
            opacity: 1;
          }
        }
        .typing-dot {
          width: 12px;
          height: 12px;
          background: ${primaryColor};
          border-radius: 50%;
          animation: typing 1.4s infinite ease-in-out;
        }
        .typing-dot:nth-child(2) {
          animation-delay: 0.2s;
        }
        .typing-dot:nth-child(3) {
          animation-delay: 0.4s;
        }
      `}</style>
    </div>
  );
}