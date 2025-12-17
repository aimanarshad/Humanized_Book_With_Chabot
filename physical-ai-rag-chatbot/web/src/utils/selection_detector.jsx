/**
 * Utility for detecting and handling text selection for the Physical AI RAG chatbot
 */

class SelectionDetector {
  constructor() {
    this.selectedText = '';
    this.onSelectionChange = null;
  }

  /**
   * Initialize the selection detection
   * @param {Function} callback - Function to call when selection changes
   */
  init(callback) {
    this.onSelectionChange = callback;

    // Add event listeners
    document.addEventListener('mouseup', this.handleSelection.bind(this));
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        this.clearSelection();
      }
    });
  }

  /**
   * Handle text selection event
   */
  handleSelection() {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    if (text !== this.selectedText) {
      this.selectedText = text;
      if (this.onSelectionChange) {
        this.onSelectionChange(text);
      }
    }
  }

  /**
   * Get currently selected text
   */
  getSelectedText() {
    return this.selectedText;
  }

  /**
   * Clear the current selection
   */
  clearSelection() {
    this.selectedText = '';
    if (this.onSelectionChange) {
      this.onSelectionChange('');
    }
    window.getSelection().removeAllRanges();
  }

  /**
   * Get the selected text range
   */
  getSelectedRange() {
    const selection = window.getSelection();
    if (selection.rangeCount > 0) {
      return selection.getRangeAt(0);
    }
    return null;
  }

  /**
   * Highlight selected text with a custom style
   */
  highlightSelection() {
    const range = this.getSelectedRange();
    if (!range) return;

    // Create a temporary element to wrap the selection
    const span = document.createElement('span');
    span.style.backgroundColor = 'rgba(255, 255, 0, 0.3)';
    span.style.borderRadius = '2px';

    try {
      range.surroundContents(span);
    } catch (e) {
      // If surrounding fails, we'll just work with the selection as is
      console.log('Could not wrap selection, working with existing selection');
    }
  }

  /**
   * Remove highlight from selected text
   */
  removeHighlight() {
    const highlightedSpans = document.querySelectorAll('span[style*="rgba(255, 255, 0, 0.3)"]');
    highlightedSpans.forEach(span => {
      // Extract the text content and replace the span with its text
      const textNode = document.createTextNode(span.textContent);
      span.parentNode.replaceChild(textNode, span);
    });
  }

  /**
   * Get context around the selected text
   */
  getSelectionContext(range = null) {
    if (!range) {
      range = this.getSelectedRange();
    }

    if (!range) return { before: '', selection: this.selectedText, after: '' };

    const startContainer = range.startContainer;
    const endContainer = range.endContainer;

    let before = '';
    let after = '';

    // Get text before selection
    if (startContainer.nodeType === Node.TEXT_NODE) {
      before = startContainer.textContent.substring(0, range.startOffset).slice(-100); // Last 100 chars
    }

    // Get text after selection
    if (endContainer.nodeType === Node.TEXT_NODE) {
      after = endContainer.textContent.substring(range.endOffset, range.endOffset + 100); // Next 100 chars
    }

    return {
      before,
      selection: this.selectedText,
      after
    };
  }

  /**
   * Get the source context (which document/page the selection is from)
   */
  getSourceContext() {
    return {
      url: window.location.href,
      title: document.title,
      element: this.getSelectedRange()?.startContainer?.parentElement?.tagName || 'unknown'
    };
  }
}

// Singleton instance
const selectionDetector = new SelectionDetector();
export default selectionDetector;

// Export for direct usage
export { SelectionDetector };