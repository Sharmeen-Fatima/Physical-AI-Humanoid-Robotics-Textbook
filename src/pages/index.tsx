import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Embodied Intelligence',
    icon: '🤖',
    description: (
      <>
        Learn how physical robots perceive and interact with the real world,
        bridging the gap between digital AI and tangible machines.
      </>
    ),
  },
  {
    title: 'Spec-Driven Development',
    icon: '🧠',
    description: (
      <>
        Master a systematic approach to building robotics systems with clear
        specifications, testable outcomes, and production-ready code.
      </>
    ),
  },
  {
    title: 'Sim-to-Real Transfer',
    icon: '⚡',
    description: (
      <>
        Deploy AI models from simulation to physical robots with confidence,
        using cutting-edge techniques for real-world performance.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureCard}>
        <div className={styles.featureIcon}>{icon}</div>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className={styles.heroTitle}>
            Build the Body. Code the Brain.
          </Heading>
          <p className={styles.heroSubtitle}>
            The World's First Spec-Driven Course on Physical AI & Humanoid Robotics.
            <br />
            <span className={styles.author}>Written by Sharmeen Fatima</span>
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning (Module 1)
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro">
              View Syllabus
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="The World's First Spec-Driven Course on Physical AI & Humanoid Robotics. Learn embodied intelligence, sim-to-real transfer, and production robotics systems.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
