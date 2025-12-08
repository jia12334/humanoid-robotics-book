import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import HeroSection from '@site/src/components/HeroSection';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HeroSection
        title="The Power of AI in a Digital World"
        subtitle="Artificial Intelligence is revolutionizing the way we live and work. From automating repetitive tasks to improving healthcare and business operations, AI is driving efficiency and innovation."
        primaryButtonText="Get Started"
        primaryButtonLink="/docs/intro"
        secondaryButtonText="Learn More"
        secondaryButtonLink="/docs/category/tutorial"
      />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
