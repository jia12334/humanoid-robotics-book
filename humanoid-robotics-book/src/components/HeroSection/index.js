import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function HeroSection({
  title = 'The Power of AI in a Digital World',
  subtitle = 'Artificial Intelligence is revolutionizing the way we live and work. From automating repetitive tasks to improving healthcare and business operations, AI is driving efficiency and innovation.',
  primaryButtonText = 'Get Started',
  primaryButtonLink = '/docs/intro',
  secondaryButtonText = 'Learn More',
  secondaryButtonLink = '/docs/category/tutorial',
}) {
  return (
    <header className={styles.hero}>
      <div className={styles.heroBackground} />
      <div className={styles.heroOverlay} />
      <div className={styles.heroContent}>
        <h1 className={styles.heroTitle}>{title}</h1>
        <p className={styles.heroSubtitle}>{subtitle}</p>
        <div className={styles.heroButtons}>
          <Link className={styles.primaryButton} to={primaryButtonLink}>
            {primaryButtonText}
          </Link>
          <Link className={styles.secondaryButton} to={secondaryButtonLink}>
            {secondaryButtonText}
          </Link>
        </div>
      </div>
    </header>
  );
}
