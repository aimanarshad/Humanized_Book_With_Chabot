import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';

const ChatInterface = ({ apiUrl = "http://localhost:8000" }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      setSelectedText(selectedText);
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Function to add a message to the chat
  const addMessage = (sender, text, sources = null) => {
    const newMessage = {
      id: Date.now(),
      sender,
      text,
      timestamp: new Date().toISOString(),
      sources
    };
    setMessages(prev => [...prev, newMessage]);
  };

  // Function to call the API and get response
  const getResponse = async (question, useSelectedText = false) => {
    setIsLoading(true);
    try {
      let response;
      if (useSelectedText && selectedText) {
        // Use the selection-based endpoint
        response = await axios.post(`${apiUrl}/v1/query-with-selection`, {
          question,
          selected_text: selectedText,
          conversation_id: conversationId
        }, {
          headers: {
            'X-API-Key': process.env.REACT_APP_PHYSICAL_AI_API_KEY || '4c3eecd9308b619594f73ee4c852a02330c68a37063fdf74',
            'Content-Type': 'application/json'
          }
        });
      } else {
        // Use the basic query endpoint
        response = await axios.post(`${apiUrl}/v1/query`, {
          question,
          conversation_id: conversationId
        }, {
          headers: {
            'X-API-Key': process.env.REACT_APP_PHYSICAL_AI_API_KEY || '4c3eecd9308b619594f73ee4c852a02330c68a37063fdf74',
            'Content-Type': 'application/json'
          }
        });
      }

      // Update conversation ID if it was created
      if (response.data.conversation_id && !conversationId) {
        setConversationId(response.data.conversation_id);
      }

      // Add bot response with sources
      addMessage('bot', response.data.answer, response.data.sources || []);
    } catch (error) {
      console.error('Error getting response:', error);
      addMessage('bot', 'Sorry, I encountered an error. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  // Handle form submission
  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() === '') return;

    // Add user message
    addMessage('user', inputValue);

    // Get response from API
    getResponse(inputValue, selectedText !== '');

    // Clear input
    setInputValue('');
  };

  // Handle feedback
  const handleFeedback = async (messageId, rating) => {
    try {
      await axios.post(`${apiUrl}/v1/feedback`, {
        message_id: messageId,
        rating
      }, {
        headers: {
          'X-API-Key': process.env.REACT_APP_PHYSICAL_AI_API_KEY || '',
          'Content-Type': 'application/json'
        }
      });
      console.log(`Feedback submitted: ${rating}`);
    } catch (error) {
      console.error('Error submitting feedback:', error);
    }
  };

  return (
    <div className="chat-container">
      <div className="chat-header">
        <h3>Physical AI Assistant</h3>
        {selectedText && (
          <div className="selected-text-indicator">
            Using selected text: "{selectedText.substring(0, 50)}..."
          </div>
        )}
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hi! I'm your Physical AI & Humanoid Robotics assistant.</p>
            <p>Ask me about ROS 2, Gazebo, NVIDIA Isaac, Vision-Language-Action systems, and more!</p>
            <p>You can also select text on the page and ask questions about it specifically.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
            >
              <div className="message-content">
                {message.text}
              </div>

              {message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  <strong>Sources:</strong>
                  <ul>
                    {message.sources.map((source, idx) => (
                      <li key={idx}>{source}</li>
                    ))}
                  </ul>
                </div>
              )}

              {message.sender === 'bot' && (
                <div className="feedback-options">
                  <button
                    onClick={() => handleFeedback(message.id, 'positive')}
                    className="feedback-btn positive"
                  >
                    👍
                  </button>
                  <button
                    onClick={() => handleFeedback(message.id, 'negative')}
                    className="feedback-btn negative"
                  >
                    👎
                  </button>
                </div>
              )}
            </div>
          ))
        )}

        {isLoading && (
          <div className="message bot-message">
            <div className="typing-indicator">
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form className="chat-input-form" onSubmit={handleSubmit}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder={selectedText
            ? "Ask about the selected text..."
            : "Ask about Physical AI, ROS 2, Isaac, VLA, etc..."}
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading || inputValue.trim() === ''}>
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatInterface;