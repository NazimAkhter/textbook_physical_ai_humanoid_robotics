import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ModuleCards from '@site/src/components/ModuleCards';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.hero}>
      <div className="container">
        <Heading as="h1" className={styles.heroTitle}>
          Physical AI & Humanoid Robotics Book
        </Heading>
        <img
          src="/img/hero-robot.png"
          alt="Robot illustration"
          className={styles.heroImage}
        />
        <div className={styles.heroButtons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/introduction">
            Get Started
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="/docs/category/module-1-ros-2-nervous-system">
            View Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Master humanoid robotics with ROS 2, digital twins, and Vision-Language-Action systems. A comprehensive guide to Physical AI.">
      <HomepageHeader />
      <main>
        <ModuleCards />
      </main>
    </Layout>
  );
}
