import React from 'react';
import ReactDOM from 'react-dom/client';
import ChatInterface from '../components/ChatInterface';

// CSS styles for the chat interface
const styles = `
  .chat-container {
    display: flex;
    flex-direction: column;
    height: 100vh;
    width: 440px;
    max-width: 100%;
    background: linear-gradient(to bottom, #f8fafc 0%, #f0f4f8 100%);
    border-radius: 18px;
    box-shadow: 
      0 20px 40px rgba(0, 0, 0, 0.1),
      0 8px 20px rgba(0, 0, 0, 0.06);
    overflow: hidden;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
    border: 1px solid rgba(226, 232, 240, 0.7);
  }

  .chat-header {
    background: linear-gradient(135deg, #4f46e5 0%, #7c3aed 80%, #a855f7 100%);
    color: white;
    padding: 24px 20px 20px;
    text-align: center;
    position: relative;
    box-shadow: 0 4px 20px rgba(79, 70, 229, 0.4);
  }

  .chat-header h3 {
    margin: 0;
    font-size: 1.3em;
    font-weight: 700;
    letter-spacing: -0.5px;
  }

  .selected-text-indicator {
    background: rgba(255, 255, 255, 0.25);
    backdrop-filter: blur(8px);
    border-radius: 10px;
    padding: 8px 14px;
    margin-top: 12px;
    font-size: 0.85em;
    max-width: 90%;
    margin-left: auto;
    margin-right: auto;
    border: 1px solid rgba(255, 255, 255, 0.2);
    box-shadow: 0 2px 8px rgba(0,0,0,0.1);
  }

  .chat-messages {
    flex: 1;
    overflow-y: auto;
    padding: 24px 20px;
    display: flex;
    flex-direction: column;
    gap: 18px;
    background: transparent;
  }

  .chat-messages::-webkit-scrollbar {
    width: 6px;
  }

  .chat-messages::-webkit-scrollbar-thumb {
    background: rgba(100, 116, 139, 0.3);
    border-radius: 3px;
  }

  .welcome-message {
    text-align: center;
    color: #475569;
    line-height: 1.7;
    font-size: 15px;
    background: rgba(255, 255, 255, 0.7);
    padding: 24px;
    border-radius: 20px;
    backdrop-filter: blur(10px);
    box-shadow: 0 8px 25px rgba(0, 0, 0, 0.08);
    max-width: 90%;
    margin: 40px auto auto;
    border: 1px solid rgba(226, 232, 240, 0.8);
  }

  .welcome-message p {
    margin: 12px 0;
  }

  .message {
    max-width: 85%;
    animation: fadeIn 0.4s ease-out;
  }

  @keyframes fadeIn {
    from { opacity: 0; transform: translateY(12px); }
    to { opacity: 1; transform: translateY(0); }
  }

  .user-message {
    align-self: flex-end;
  }

  .bot-message {
    align-self: flex-start;
  }

  .message-content {
    padding: 15px 20px;
    border-radius: 22px;
    line-height: 1.6;
    font-size: 15.5px;
    box-shadow: 0 3px 12px rgba(0, 0, 0, 0.08);
    word-wrap: break-word;
  }

  .user-message .message-content {
    background: linear-gradient(135deg, #4f46e5 0%, #7c3aed 100%);
    color: white;
    border-bottom-right-radius: 8px;
  }

  .bot-message .message-content {
    background: white;
    color: #1e293b;
    border-bottom-left-radius: 8px;
    border: 1px solid #e2e8f0;
  }

  .message-sources {
    margin-top: 10px;
    font-size: 0.82em;
    color: #64748b;
  }

  .message-sources strong {
    color: #475569;
  }

  .message-sources ul {
    margin: 8px 0 0;
    padding-left: 20px;
  }

  .message-sources li {
    margin: 4px 0;
    line-height: 1.4;
  }

  .feedback-options {
    margin-top: 10px;
    opacity: 0;
    transition: opacity 0.25s ease;
    text-align: right;
  }

  .bot-message:hover .feedback-options {
    opacity: 1;
  }

  .feedback-btn {
    background: none;
    border: 1px solid rgba(100, 100, 100, 0.2);
    border-radius: 50%;
    width: 36px;
    height: 36px;
    font-size: 1.1em;
    cursor: pointer;
    transition: all 0.2s ease;
    margin-left: 8px;
  }

  .feedback-btn:hover {
    transform: scale(1.15);
    background: rgba(79, 70, 229, 0.1);
    border-color: #4f46e5;
  }

  .feedback-btn.positive:hover {
    background: rgba(34, 197, 94, 0.15);
    border-color: #22c55e;
  }

  .feedback-btn.negative:hover {
    background: rgba(239, 68, 68, 0.15);
    border-color: #ef4444;
  }

  .typing-indicator {
    padding: 15px 20px;
    background: white;
    border-radius: 22px;
    border-bottom-left-radius: 8px;
    display: inline-block;
    box-shadow: 0 3px 12px rgba(0, 0, 0, 0.08);
    border: 1px solid #e2e8f0;
  }

  .typing-indicator span {
    display: inline-block;
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: #94a3b8;
    margin: 0 4px;
    animation: typingBounce 1.5s infinite ease-in-out;
  }

  .typing-indicator span:nth-child(1) { animation-delay: 0s; }
  .typing-indicator span:nth-child(2) { animation-delay: 0.2s; }
  .typing-indicator span:nth-child(3) { animation-delay: 0.4s; }

  @keyframes typingBounce {
    0%, 80%, 100% { transform: translateY(0); opacity: 0.5; }
    40% { transform: translateY(-10px); opacity: 1; }
  }

  .chat-input-form {
    padding: 20px;
    background: rgba(255, 255, 255, 0.75);
    backdrop-filter: blur(16px);
    border-top: 1px solid rgba(226, 232, 240, 0.8);
  }

  .chat-input-wrapper {
    display: flex;
    align-items: center;
    background: white;
    border-radius: 30px;
    padding: 8px 12px;
    box-shadow: 0 6px 20px rgba(0, 0, 0, 0.12);
    border: 1px solid #e2e8f0;
    transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  }

  .chat-input-wrapper:focus-within {
    transform: translateY(-3px);
    box-shadow: 0 12px 30px rgba(79, 70, 229, 0.25);
    border-color: #4f46e5;
  }

  .chat-input-form input {
    flex: 1;
    padding: 16px 20px;
    border: none;
    font-size: 16px;
    outline: none;
    background: transparent;
    color: #1e293b;
  }

  .chat-input-form input::placeholder {
    color: #94a3b8;
    font-style: italic;
  }

  .chat-input-form input:disabled {
    opacity: 0.6;
  }

  .chat-input-form button {
    padding: 14px 28px;
    background: linear-gradient(135deg, #4f46e5 0%, #7c3aed 100%);
    color: white;
    border: none;
    border-radius: 26px;
    cursor: pointer;
    font-weight: 600;
    font-size: 15px;
    transition: all 0.3s ease;
    min-width: 90px;
  }

  .chat-input-form button:hover:not(:disabled) {
    transform: translateY(-2px);
    box-shadow: 0 8px 20px rgba(79, 70, 229, 0.4);
  }

  .chat-input-form button:disabled {
    background: #cbd5e1;
    cursor: not-allowed;
    transform: none;
    box-shadow: none;
  }
`;
// Create a style element and add the styles
const styleElement = document.createElement('style');
styleElement.textContent = styles;
document.head.appendChild(styleElement);

// Main App component
function App() {
  return (
    <div className="chat-container">
      <ChatInterface />
    </div>
  );
}

// Render the app to the DOM
const container = document.getElementById('root') || document.body;
const root = ReactDOM.createRoot(container);
root.render(<App />);

export default App;