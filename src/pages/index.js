import React from 'react';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { usePluginData } from '@docusaurus/useGlobalData';
import Link from '@docusaurus/Link';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const history = useHistory();

  const handleStartReading = () => {
    history.push('/intro');
    history.push('/blog');
  };

  return (
    <div style={{ padding: '4rem 0', textAlign: 'center', position: 'relative', overflow: 'hidden' }}>
      <div style={{
        position: 'absolute',
        top: 0, left: 0,
        width: '100%', height: '100%',
        background: 'linear-gradient(45deg, #6a11cb, #2575fc)',
        zIndex: -1
      }}></div>
      <div className="container">
        <h1 style={{ fontSize: '4rem', fontWeight: 'bold', color: 'white', textShadow: '2px 2px 4px rgba(0,0,0,0.3)' }}>
          {siteConfig.title}
        </h1>
        <p style={{ fontSize: '1.5rem', color: 'white', opacity: 0.9 }}>
          {siteConfig.tagline}
        </p>
        <div style={{ marginTop: '2rem' }}>
          
<Link to="/intro">
  <button
    style={{
      padding: '1rem 2rem',
      fontSize: '1.2rem',
      fontWeight: 'bold',
      color: 'white',
      background: '#ff4081',
      border: 'none',
      borderRadius: '5px',
      cursor: 'pointer',
      boxShadow: '0 4px 6px rgba(0,0,0,0.1)',
      transition: 'all 0.3s ease'
    }}
  >
    Start Reading
  </button>
</Link>
        </div>
      </div>
    </div>
  );
}



export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A Comprehensive Guide to Embodied Intelligence"
    >
      
      <HomepageHeader />
      <main>
      </main>
    </Layout>
  );
}
