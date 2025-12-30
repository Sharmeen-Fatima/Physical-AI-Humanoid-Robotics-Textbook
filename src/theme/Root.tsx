/**
 * Docusaurus Root Component
 *
 * Wraps the entire Docusaurus app to add global components like the chatbot.
 */

import React from 'react';

// Import the original Root component
// @ts-ignore
import OriginalRoot from '@theme-original/Root';

// Error boundary for chatbot
class ChatbotErrorBoundary extends React.Component<
  { children: React.ReactNode },
  { hasError: boolean }
> {
  constructor(props: { children: React.ReactNode }) {
    super(props);
    this.state = { hasError: false };
  }

  static getDerivedStateFromError() {
    return { hasError: true };
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo) {
    console.error('Chatbot Error:', error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      return null; // Don't render chatbot if it errors
    }
    return this.props.children;
  }
}

// Lazy load the chatbot to prevent blocking page load
const ChatbotWidget = React.lazy(() =>
  import('../components/Chatbot/ChatbotWidget').then((module) => ({
    default: module.default || module.ChatbotWidget,
  }))
);

export default function Root(props: any) {
  return (
    <>
      <OriginalRoot {...props} />
      <ChatbotErrorBoundary>
        <React.Suspense fallback={null}>
          <ChatbotWidget
            config={{
              topK: 5,
              threshold: 0.70,
              maxQueryLength: 500,
              position: 'bottom-right',
              theme: 'auto',
            }}
          />
        </React.Suspense>
      </ChatbotErrorBoundary>
    </>
  );
}
