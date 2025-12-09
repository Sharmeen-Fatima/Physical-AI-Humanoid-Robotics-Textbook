import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Physical AI & Humanoid Robotics Textbook Sidebar
 * Module-based structure for progressive learning
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: '🤖 Module 1: The Robotic Nervous System',
      collapsible: true,
      collapsed: false,
      items: [
        'module-01-ros2/nodes',
        'module-01-ros2/python-agents',
      ],
    },
    {
      type: 'category',
      label: '🎮 Module 2: The Digital Twin',
      collapsible: true,
      collapsed: true,
      items: [
        'module-02-digital-twin/gazebo',
        'module-02-digital-twin/unity',
      ],
    },
    {
      type: 'category',
      label: '🧠 Module 3: The AI-Robot Brain',
      collapsible: true,
      collapsed: true,
      items: [
        'module-03-robot-brain/isaac-sim',
        'module-03-robot-brain/nav2',
      ],
    },
    {
      type: 'category',
      label: '💬 Module 4: Vision-Language-Action',
      collapsible: true,
      collapsed: true,
      items: [
        'module-04-vla/whisper',
        'module-04-vla/llm-control',
      ],
    },
    {
      type: 'category',
      label: '⚡ Capstone Project',
      collapsible: true,
      collapsed: true,
      items: [
        'module-05-capstone/project-brief',
      ],
    },
  ],
};

export default sidebars;
