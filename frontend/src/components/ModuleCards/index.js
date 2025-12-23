import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const modules = [
  {
    number: 1,
    title: 'ROS 2 as the Robotic Nervous System',
    description: 'Learn how ROS 2 enables communication between sensors, controllers, and actuators in humanoid robots.',
    href: '/docs/category/module-1-ros-2-nervous-system',
    status: 'available',
  },
  {
    number: 2,
    title: 'Digital Twin with Gazebo & Unity',
    description: 'Build simulation environments that mirror real-world physics for safe robot development and testing.',
    href: '/docs/category/module-2-digital-twins',
    status: 'available',
  },
  {
    number: 3,
    title: 'The AI-Robot Brain (NVIDIA Isaac)',
    description: 'Learn how NVIDIA Isaac provides photorealistic simulation, GPU-accelerated perception, and autonomous navigation for humanoid robots.',
    href: '/docs/category/module-3-ai-robot-brain',
    status: 'available',
  },
  {
    number: 4,
    title: 'Vision-Language-Action Models',
    description: 'Implement VLA systems that translate natural language commands into robot actions.',
    href: '/docs/category/module-4-vla-systems',
    status: 'available',
  },
  {
    number: 5,
    title: 'End-to-End Humanoid Integration',
    description: 'Bring together all components to create a fully functional humanoid robot system.',
    href: '/docs/module-05-integration',
    status: 'coming-soon',
  },
];

function ModuleCard({ number, title, description, href, status }) {
  const isAvailable = status === 'available';

  const cardContent = (
    <>
      <div className={styles.moduleNumber}>Module {number}</div>
      <h3 className={styles.moduleTitle}>{title}</h3>
      <p className={styles.moduleDescription}>{description}</p>
      {!isAvailable && <span className={styles.comingSoonBadge}>Coming Soon</span>}
    </>
  );

  if (isAvailable) {
    return (
      <Link to={href} className={`${styles.moduleCard} ${styles.moduleCardAvailable}`}>
        {cardContent}
      </Link>
    );
  }

  return (
    <div className={`${styles.moduleCard} ${styles.moduleCardComingSoon}`}>
      {cardContent}
    </div>
  );
}

export default function ModuleCards() {
  return (
    <section className={styles.moduleCards}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <div className={styles.moduleGrid}>
          {modules.map((module) => (
            <ModuleCard key={module.number} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}
