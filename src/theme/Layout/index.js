import React from 'react';
import Layout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot';
import styles from './styles.module.css';

export default function LayoutWrapper(props) {
  return (
    <div className={styles.wrapper_src_theme_Layout}>
      <Layout {...props}>
        {props.children}
      </Layout>
      <div className={styles.floatingChatbot}>
        <Chatbot />
      </div>
    </div>
  );
}
