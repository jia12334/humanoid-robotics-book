/**
 * Root component wrapper for Docusaurus.
 *
 * This wraps the entire Docusaurus app and allows us to inject
 * global components like the ChatWidget.
 *
 * @see https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React from 'react';
import ChatWidget from '../components/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
