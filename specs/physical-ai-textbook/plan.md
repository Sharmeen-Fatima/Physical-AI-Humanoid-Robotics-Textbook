# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `physical-ai-textbook` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/physical-ai-textbook/spec.md`

## Summary

Create a comprehensive, production-ready Docusaurus v3 educational website for teaching Physical AI and Humanoid Robotics. The textbook will cover 4 core modules (ROS 2, Simulation, Isaac Sim/Nav2, VLA) plus a capstone project, with 30+ Mermaid.js diagrams, 20+ runnable Python code examples, and a cyber-physical design aesthetic using electric blue, neon purple, and deep slate colors.

**Technical Approach**:
- Docusaurus v3 with TypeScript for static site generation
- Custom sidebar structure grouping 5 modules with 20 total chapters
- Mermaid.js for all architecture and data flow diagrams
- Prism.js syntax highlighting for Python/YAML/XML code
- Cyber-physical custom CSS theme (already implemented)
- MDX for interactive components (code tabs, admonitions)

## Technical Context

**Language/Version**: TypeScript 5.x, Node.js 18+, Docusaurus 3.x
**Primary Dependencies**:
- `@docusaurus/core` ^3.0.0
- `@docusaurus/preset-classic` ^3.0.0
- `@docusaurus/theme-mermaid` (for diagrams)
- `prism-react-renderer` (for code highlighting)
- `clsx` (for CSS utilities)

**Storage**: Static Markdown/MDX files in `docs/` directory
**Testing**: Manual review, build verification (`npm run build`), link checking
**Target Platform**: Web (static site), deployed via GitHub Pages or Vercel
**Project Type**: Documentation site (Docusaurus)
**Performance Goals**:
- Page load <2s on 3G
- Lighthouse score >90
- Build time <60s for 20 pages

**Constraints**:
- All diagrams must render in Mermaid.js (no external image dependencies)
- Code snippets must be copy-pasteable and runnable
- Mobile-responsive design (critical for on-device learning)
- Dark mode must be default (cyber-physical aesthetic)

**Scale/Scope**:
- 20 chapter pages (Markdown/MDX)
- 1 landing page (React/TSX)
- 30+ Mermaid diagrams
- 20+ code examples
- 5 module categories in sidebar

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Design Philosophy**: "The Cyber-Physical Look"
- Primary color: Electric Blue/Cyan (#00d9ff) ✅ IMPLEMENTED
- Background: Deep Slate/Black (#0a0e1a) ✅ IMPLEMENTED
- Accents: Neon Purple (#b366ff) ✅ IMPLEMENTED
- Font: Inter (clean sans-serif) ✅ IMPLEMENTED

✅ **Component Requirements**:
- Landing page hero: "Build the Body. Code the Brain." ✅ IMPLEMENTED
- Feature cards: 🤖 🧠 ⚡ ✅ IMPLEMENTED
- Navbar: Glass-morphism effect ✅ IMPLEMENTED
- Admonitions: System Alert style ✅ IMPLEMENTED

✅ **Learning Experience Requirements**:
- Visuals: Mermaid.js for every technical concept ⏳ TO IMPLEMENT
- Code: Python snippets for all topics ⏳ TO IMPLEMENT
- Tone: Visionary, co-learning style ⏳ TO IMPLEMENT

**Constitution Violations**: None

## Project Structure

### Documentation (this feature)

```text
specs/physical-ai-textbook/
├── spec.md              # Feature specification (COMPLETE)
├── plan.md              # This file (/sp.plan output)
├── tasks.md             # Phase 2 output (/sp.tasks - NOT YET CREATED)
└── diagrams/            # Mermaid diagram source files (optional reference)
```

### Source Code (Docusaurus Structure)

```text
docs/
├── intro.md                                    # Landing page / Preface
│
├── module-01-ros2/                             # Module 1: The Robotic Nervous System
│   ├── _category_.json                         # Sidebar label config
│   ├── 01-biological-analogy.md                # Chapter 1.1
│   ├── 02-first-node.md                        # Chapter 1.2
│   ├── 03-services-actions.md                  # Chapter 1.3
│   ├── 04-urdf-basics.md                       # Chapter 1.4
│   └── 05-coordinate-frames.md                 # Chapter 1.5
│
├── module-02-digital-twin/                     # Module 2: Simulation
│   ├── _category_.json
│   ├── 01-why-simulate.md                      # Chapter 2.1
│   ├── 02-gazebo-fundamentals.md               # Chapter 2.2
│   ├── 03-physics-sensors.md                   # Chapter 2.3
│   ├── 04-unity-hri.md                         # Chapter 2.4
│   └── 05-sim-to-real.md                       # Chapter 2.5
│
├── module-03-ai-brain/                         # Module 3: Isaac Sim & Nav2
│   ├── _category_.json
│   ├── 01-isaac-sim-intro.md                   # Chapter 3.1
│   ├── 02-synthetic-data.md                    # Chapter 3.2
│   ├── 03-nav2-architecture.md                 # Chapter 3.3
│   ├── 04-slam-mapping.md                      # Chapter 3.4
│   └── 05-path-planning.md                     # Chapter 3.5
│
├── module-04-vla/                              # Module 4: Vision-Language-Action
│   ├── _category_.json
│   ├── 01-vla-revolution.md                    # Chapter 4.1
│   ├── 02-voice-whisper.md                     # Chapter 4.2
│   ├── 03-llm-parsing.md                       # Chapter 4.3
│   ├── 04-action-servers.md                    # Chapter 4.4
│   └── 05-safety-validation.md                 # Chapter 4.5
│
└── module-05-capstone/                         # Capstone Project
    ├── _category_.json
    ├── 01-project-overview.md                  # Project requirements
    ├── 02-integration-guide.md                 # Step-by-step integration
    └── 03-rubric-assessment.md                 # Self-assessment rubric

static/
├── img/
│   ├── logo.svg                                # Site logo (TO CREATE)
│   ├── favicon.ico                             # Favicon (TO UPDATE)
│   └── social-card.jpg                         # Social media preview (TO CREATE)

src/
├── components/
│   ├── HomepageFeatures/                       # ALREADY IMPLEMENTED (in index.tsx)
│   ├── CodeBlock/                              # Custom code block wrapper (OPTIONAL)
│   └── InteractiveDiagram/                     # Interactive Mermaid wrapper (OPTIONAL)
│
├── css/
│   └── custom.css                              # Cyber-physical theme (COMPLETE)
│
└── pages/
    ├── index.tsx                               # Landing page (COMPLETE)
    └── index.module.css                        # Landing styles (COMPLETE)

# Configuration files (root)
docusaurus.config.ts                            # Site config (UPDATED)
sidebars.ts                                     # Sidebar structure (TO UPDATE)
package.json                                    # Dependencies (EXISTING)
tsconfig.json                                   # TypeScript config (EXISTING)
```

**Structure Decision**: Standard Docusaurus documentation site with manual sidebar configuration. Each module is a top-level category with 5 chapters. The `_category_.json` files provide sidebar labels and ordering. All content is Markdown/MDX for simplicity and maintainability.

## Complexity Tracking

> **No violations - standard Docusaurus documentation structure**

N/A - This implementation follows Docusaurus best practices and the constitution requirements.

---

## Phase 0: Research & Discovery

### Existing Codebase Analysis

**Current State**:
- ✅ Docusaurus 3.x initialized with TypeScript
- ✅ Cyber-physical custom CSS implemented (`src/css/custom.css`)
- ✅ Landing page redesigned (`src/pages/index.tsx`, `src/pages/index.module.css`)
- ✅ Site config updated (`docusaurus.config.ts`)
- ✅ Default docs structure exists (`docs/intro.md`, `docs/tutorial-basics/`, `docs/tutorial-extras/`)
- ⏳ Sidebar uses autogenerated structure (needs manual configuration)
- ⏳ No module-specific content created yet

**Dependencies Verified**:
```json
{
  "@docusaurus/core": "^3.x.x",
  "@docusaurus/preset-classic": "^3.x.x",
  "@docusaurus/theme-mermaid": "NEEDS INSTALLATION",
  "react": "^18.x.x",
  "typescript": "^5.x.x"
}
```

**Action Items**:
1. Install `@docusaurus/theme-mermaid` for diagram support
2. Remove default tutorial folders (`tutorial-basics/`, `tutorial-extras/`)
3. Create module directory structure
4. Update `sidebars.ts` with manual configuration
5. Update `intro.md` with textbook preface

---

## Phase 1: Architecture & Design

### 1.1 Content Architecture

#### Module Structure Design

Each module follows this pattern:

```markdown
# [Module Title]

**Learning Objectives**:
- Objective 1
- Objective 2
- Objective 3

**Prerequisites**: [Prior module knowledge required]

**Estimated Time**: [X hours]

---

## [Section 1: Concept Introduction]

[Explanation using co-learning tone]

:::tip Key Insight
[Important concept highlighted]
:::

### Visual: [Diagram Title]

```mermaid
[Mermaid diagram code]
```

### Code Example: [Example Title]

```python
# [Runnable Python code with comments]
```

**Try It Yourself**:
- Modify X to see Y
- Experiment with Z

---

## [Section 2: Hands-On Exercise]

[Step-by-step instructions]

**Acceptance Criteria**:
- [ ] Criterion 1
- [ ] Criterion 2

---

## Summary

[Recap of key points]

**Next Steps**: [Link to next chapter]
```

#### Sidebar Configuration

Manual sidebar in `sidebars.ts`:

```typescript
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: '🤖 Module 1: The Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-01-ros2/01-biological-analogy',
        'module-01-ros2/02-first-node',
        'module-01-ros2/03-services-actions',
        'module-01-ros2/04-urdf-basics',
        'module-01-ros2/05-coordinate-frames',
      ],
    },
    {
      type: 'category',
      label: '🎮 Module 2: The Digital Twin (Simulation)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-02-digital-twin/01-why-simulate',
        'module-02-digital-twin/02-gazebo-fundamentals',
        'module-02-digital-twin/03-physics-sensors',
        'module-02-digital-twin/04-unity-hri',
        'module-02-digital-twin/05-sim-to-real',
      ],
    },
    {
      type: 'category',
      label: '🧠 Module 3: The AI-Robot Brain (Isaac Sim & Nav2)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-03-ai-brain/01-isaac-sim-intro',
        'module-03-ai-brain/02-synthetic-data',
        'module-03-ai-brain/03-nav2-architecture',
        'module-03-ai-brain/04-slam-mapping',
        'module-03-ai-brain/05-path-planning',
      ],
    },
    {
      type: 'category',
      label: '💬 Module 4: Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-04-vla/01-vla-revolution',
        'module-04-vla/02-voice-whisper',
        'module-04-vla/03-llm-parsing',
        'module-04-vla/04-action-servers',
        'module-04-vla/05-safety-validation',
      ],
    },
    {
      type: 'category',
      label: '⚡ Capstone Project: The Autonomous Humanoid',
      collapsible: true,
      collapsed: true,
      items: [
        'module-05-capstone/01-project-overview',
        'module-05-capstone/02-integration-guide',
        'module-05-capstone/03-rubric-assessment',
      ],
    },
  ],
};
```

### 1.2 Diagram Specifications

#### Mermaid Configuration

Enable in `docusaurus.config.ts`:

```typescript
module.exports = {
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],
};
```

#### Diagram Inventory (30+ diagrams)

**Module 1 (6 diagrams)**:
1. **ROS 2 Architecture** (Graph)
   - Nodes, topics, DDS middleware layer
2. **Nervous System Analogy** (Flowchart)
   - Brain → Spinal Cord → Neurons mapped to ROS components
3. **Publisher-Subscriber Flow** (Sequence)
   - Message publication and subscription lifecycle
4. **Service Call Sequence** (Sequence)
   - Client request → Server processing → Response
5. **Action Feedback Loop** (Flowchart)
   - Goal → Feedback → Result with cancellation
6. **TF Tree Example** (Graph)
   - world → odom → base_link → camera_link

**Module 2 (6 diagrams)**:
1. **Sim-to-Real Pipeline** (Flowchart)
   - Design → Simulate → Validate → Deploy
2. **Gazebo Architecture** (Graph)
   - Physics engine, plugins, ROS bridge
3. **Sensor Data Flow** (Sequence)
   - LiDAR plugin → `/scan` topic → SLAM node
4. **Unity-ROS Integration** (Graph)
   - Unity engine ↔ ROS-TCP-Connector ↔ ROS 2
5. **Domain Randomization** (Flowchart)
   - Scene → Randomize (lighting, textures, objects) → Render
6. **Validation Checklist** (Flowchart)
   - Sim parameters → Compare with real → Adjust → Retest

**Module 3 (8 diagrams)**:
1. **Isaac Sim Architecture** (Graph)
   - Omniverse → RTX ray tracing → ROS 2 bridge
2. **Synthetic Data Pipeline** (Flowchart)
   - Replicator → Scene randomization → Render → Export labels
3. **Nav2 Full Stack** (Flowchart)
   - Sensors → Costmap → Planner → Controller → cmd_vel
4. **Costmap Layers** (Graph)
   - Static map + Obstacle layer + Inflation layer
5. **SLAM Loop** (Flowchart)
   - Scan → Match → Update pose graph → Build map
6. **Behavior Tree Example** (Flowchart)
   - Navigate → Check battery → Retry or Abort
7. **Planner Comparison** (Table/Flowchart)
   - A* vs. RRT* vs. DWB (speed, smoothness, safety)
8. **Costmap Inflation** (Diagram)
   - Obstacle → Inscribed radius → Inflation radius

**Module 4 (6 diagrams)**:
1. **VLA Pipeline Full** (Flowchart)
   - Voice → Whisper → LLM → ROS 2 Action → Robot
2. **Whisper ASR Workflow** (Flowchart)
   - Audio capture → Model inference → Text output
3. **LLM Command Parsing** (Sequence)
   - Prompt + Command → LLM → Structured JSON
4. **Action Server Lifecycle** (State Machine)
   - Idle → Executing → Succeeded/Aborted/Canceled
5. **Safety Validation Layer** (Flowchart)
   - LLM output → Whitelist check → Bounds check → Execute/Reject
6. **Error Recovery Tree** (Flowchart)
   - Invalid command → Retry with clarification → Timeout → Abort

**Capstone (4 diagrams)**:
1. **Full System Integration** (Graph)
   - All modules connected: Voice → Nav2 → Execution
2. **End-to-End Data Flow** (Sequence)
   - User speaks → Robot completes task
3. **State Machine** (State Diagram)
   - Listening → Planning → Navigating → Executing → Done
4. **Deployment Checklist** (Flowchart)
   - Dependencies → Build → Test → Deploy

### 1.3 Code Snippet Architecture

#### Code Block Standards

All code examples must include:
1. **Language tag**: `python`, `yaml`, `xml`, `bash`
2. **Title comment**: `# Example: Publisher Node`
3. **Inline comments**: Explain every 3-5 lines
4. **Type hints**: For Python functions
5. **Imports**: Always show required imports
6. **Runnable**: Can be copy-pasted and executed

Example template:

````markdown
### Code Example: ROS 2 Publisher Node

```python
# Example: Minimal ROS 2 publisher in Python
# File: talker.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    """Publishes messages to the 'chatter' topic."""

    def __init__(self) -> None:
        super().__init__('talker')
        # Create publisher: topic='chatter', message type=String, queue size=10
        self.publisher = self.create_publisher(String, 'chatter', 10)
        # Timer fires every 1.0 seconds
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self) -> None:
        """Callback: publishes a message every timer tick."""
        msg = String()
        msg.data = f'Hello from Physical AI at {self.get_clock().now()}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        rclpy.spin(node)  # Keep node alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to run**:
```bash
# In terminal 1: Run the ROS 2 node
python3 talker.py

# In terminal 2: Echo the messages
ros2 topic echo /chatter
```

**Expected output**:
```
data: 'Hello from Physical AI at [timestamp]'
---
```

**Try it yourself**:
- Change the timer frequency to `0.5` for faster messages
- Modify the message content to include a counter
- Create a subscriber node to receive these messages
````

#### Code Example Inventory (20+ examples)

**Module 1**:
1. Talker (publisher) node
2. Listener (subscriber) node
3. Add two integers service server
4. Add two integers service client
5. Fibonacci action server
6. Static transform broadcaster
7. Simple URDF (2-link arm)

**Module 2**:
1. Gazebo launch file (world + robot spawn)
2. SDF file for LiDAR sensor plugin
3. Unity C# ROS subscriber
4. Domain randomization Python script
5. Physics parameter tuning YAML

**Module 3**:
1. Isaac Sim robot spawn script
2. Replicator synthetic data generation
3. Nav2 configuration YAML
4. slam_toolbox launch file
5. Custom costmap plugin (Python/C++)

**Module 4**:
1. Whisper audio transcription script
2. OpenAI LLM command parser
3. ROS 2 action server for navigation
4. Safety validator (command whitelist)
5. Full VLA integration script

**Capstone**:
1. Docker Compose for environment
2. Main integration launch file
3. State machine (Python)

### 1.4 Docusaurus Configuration Updates

#### Enable Mermaid

```typescript
// docusaurus.config.ts
import type {Config} from '@docusaurus/types';

const config: Config = {
  // ... existing config

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],

  themeConfig: {
    mermaid: {
      theme: {
        light: 'neutral',
        dark: 'dark',
      },
      options: {
        fontSize: 16,
        flowchart: {
          useMaxWidth: true,
          htmlLabels: true,
          curve: 'basis',
        },
      },
    },
  },
};
```

#### Update Prism for Python Highlighting

```typescript
// docusaurus.config.ts
themeConfig: {
  prism: {
    theme: prismThemes.github,
    darkTheme: prismThemes.dracula,
    additionalLanguages: ['python', 'bash', 'yaml', 'xml', 'json'],
  },
}
```

---

## Phase 2: Implementation Breakdown

### File Creation Checklist

#### Documentation Files (23 files)

**Intro & Landing**:
- [ ] `docs/intro.md` (rewrite with textbook preface)

**Module 1 (6 files)**:
- [ ] `docs/module-01-ros2/_category_.json`
- [ ] `docs/module-01-ros2/01-biological-analogy.md`
- [ ] `docs/module-01-ros2/02-first-node.md`
- [ ] `docs/module-01-ros2/03-services-actions.md`
- [ ] `docs/module-01-ros2/04-urdf-basics.md`
- [ ] `docs/module-01-ros2/05-coordinate-frames.md`

**Module 2 (6 files)**:
- [ ] `docs/module-02-digital-twin/_category_.json`
- [ ] `docs/module-02-digital-twin/01-why-simulate.md`
- [ ] `docs/module-02-digital-twin/02-gazebo-fundamentals.md`
- [ ] `docs/module-02-digital-twin/03-physics-sensors.md`
- [ ] `docs/module-02-digital-twin/04-unity-hri.md`
- [ ] `docs/module-02-digital-twin/05-sim-to-real.md`

**Module 3 (6 files)**:
- [ ] `docs/module-03-ai-brain/_category_.json`
- [ ] `docs/module-03-ai-brain/01-isaac-sim-intro.md`
- [ ] `docs/module-03-ai-brain/02-synthetic-data.md`
- [ ] `docs/module-03-ai-brain/03-nav2-architecture.md`
- [ ] `docs/module-03-ai-brain/04-slam-mapping.md`
- [ ] `docs/module-03-ai-brain/05-path-planning.md`

**Module 4 (6 files)**:
- [ ] `docs/module-04-vla/_category_.json`
- [ ] `docs/module-04-vla/01-vla-revolution.md`
- [ ] `docs/module-04-vla/02-voice-whisper.md`
- [ ] `docs/module-04-vla/03-llm-parsing.md`
- [ ] `docs/module-04-vla/04-action-servers.md`
- [ ] `docs/module-04-vla/05-safety-validation.md`

**Module 5 / Capstone (4 files)**:
- [ ] `docs/module-05-capstone/_category_.json`
- [ ] `docs/module-05-capstone/01-project-overview.md`
- [ ] `docs/module-05-capstone/02-integration-guide.md`
- [ ] `docs/module-05-capstone/03-rubric-assessment.md`

#### Configuration Files (2 updates)

- [ ] `sidebars.ts` (replace autogenerated with manual sidebar)
- [ ] `docusaurus.config.ts` (add Mermaid theme, update Prism languages)

#### Package Updates (1 file)

- [ ] `package.json` (install `@docusaurus/theme-mermaid`)

#### Asset Files (3 files)

- [ ] `static/img/logo.svg` (create/update robot logo)
- [ ] `static/img/favicon.ico` (update with robot icon)
- [ ] `static/img/social-card.jpg` (create social preview image)

#### Cleanup (2 deletions)

- [ ] Delete `docs/tutorial-basics/` (default demo content)
- [ ] Delete `docs/tutorial-extras/` (default demo content)

**Total**: 23 content files + 2 config updates + 1 package update + 3 assets + 2 deletions = **31 file operations**

---

## Phase 3: Content Templates

### Template 1: _category_.json

```json
{
  "label": "🤖 Module 1: The Robotic Nervous System (ROS 2)",
  "position": 2,
  "link": {
    "type": "generated-index",
    "description": "Master the middleware that connects the robot's brain to its body. Learn ROS 2 architecture using biological analogies, write your first Python nodes, and define robot geometry with URDF."
  }
}
```

### Template 2: Chapter Markdown Structure

```markdown
---
sidebar_position: [X]
title: [Chapter Title]
description: [One-sentence summary]
---

# [Chapter Title]

**Learning Objectives**:
- [Objective 1]
- [Objective 2]
- [Objective 3]

**Prerequisites**: [Prior knowledge needed]

**Estimated Time**: [X] hours

---

## Introduction

[Engaging opening using co-learning tone]

:::tip Key Concept
[Highlight important insight]
:::

---

## [Main Section 1: Concept]

[Detailed explanation with analogies]

### Visual: [Diagram Title]

```mermaid
[Mermaid diagram code]
```

[Explanation of diagram]

---

## [Main Section 2: Implementation]

[Step-by-step guide]

### Code Example: [Title]

```python
# [Runnable Python code with inline comments]
```

**How to run**:
```bash
# Commands to execute the code
```

**Expected output**:
```
[Sample output]
```

**Try it yourself**:
- [Modification 1]
- [Modification 2]

---

## Hands-On Exercise

**Challenge**: [Description of exercise]

**Acceptance Criteria**:
- [ ] [Criterion 1]
- [ ] [Criterion 2]
- [ ] [Criterion 3]

**Hints**:
- [Hint 1]
- [Hint 2]

---

## Common Pitfalls

:::warning Watch Out
[Common mistake 1]
:::

:::warning Watch Out
[Common mistake 2]
:::

---

## Summary

[Recap key points in 3-5 bullets]

**Key Takeaways**:
- [Takeaway 1]
- [Takeaway 2]
- [Takeaway 3]

**Next Steps**: In the [next chapter](./[next-file].md), we'll explore [preview].

---

## Further Reading

- [External resource 1]
- [External resource 2]
- [External resource 3]
```

---

## Phase 4: Deployment & Validation

### Build Verification Checklist

- [ ] `npm install` succeeds (all dependencies resolved)
- [ ] `npm run build` completes without errors
- [ ] No broken links (run `npm run build -- --config=docusaurus.config.ts`)
- [ ] All Mermaid diagrams render correctly
- [ ] All code blocks have syntax highlighting
- [ ] Sidebar navigation works (collapsible categories)
- [ ] Mobile responsive (test on viewport <768px)
- [ ] Dark mode renders correctly
- [ ] Lighthouse score >90 (Performance, Accessibility, SEO)

### Pre-Deployment Tests

1. **Link Checker**: Verify all internal links resolve
2. **Code Snippet Validation**: Ensure all Python code is syntactically valid
3. **Diagram Rendering**: Test Mermaid diagrams in dark/light mode
4. **Sidebar Order**: Confirm modules appear in correct sequence
5. **Search Functionality**: Test Docusaurus built-in search

### Deployment Steps

1. **Build Production Bundle**:
   ```bash
   npm run build
   ```

2. **Serve Locally for Preview**:
   ```bash
   npm run serve
   ```

3. **Deploy to GitHub Pages** (if using):
   ```bash
   GIT_USER=Sharmeen-Fatima npm run deploy
   ```

4. **Verify Deployment**:
   - Check live URL
   - Test all navigation paths
   - Verify social card preview (Twitter, LinkedIn)

---

## Risk Analysis & Mitigation

### Technical Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Mermaid diagrams don't render in build | High | Low | Test early, use `@docusaurus/theme-mermaid` official package |
| Code snippets too long for mobile | Medium | Medium | Use collapsible code blocks or tabs for long examples |
| Build time >60s with 20 pages | Low | Low | Optimize images, use static assets wisely |
| Sidebar becomes cluttered | Medium | Low | Use collapsible categories (default collapsed except Module 1) |
| Students can't run code examples | High | Medium | Provide Docker environment and `requirements.txt` |

### Content Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Diagrams become outdated with ROS 2 updates | Medium | High | Include ROS 2 version in diagram annotations |
| Code examples fail on different platforms | High | Medium | Test on Ubuntu 22.04, provide Docker fallback |
| Learning curve too steep | High | Low | Use progressive disclosure, start simple in Module 1 |
| Missing prerequisites block learners | Medium | Medium | Clearly document prerequisites at chapter start |

---

## Open Questions for Review

1. **Interactive Components**: Should we add runnable code playgrounds (e.g., CodeSandbox embeds)?
2. **Video Content**: Should diagrams link to supplementary video explanations?
3. **Glossary**: Create a separate glossary page or inline term tooltips?
4. **Exercises**: Should exercises have auto-graded solutions (e.g., GitHub Actions)?
5. **Community**: Integrate Giscus (GitHub Discussions) for comments on each page?
6. **Versioning**: Plan for ROS 2 Jazzy/Kilted updates in the future?

---

## Dependencies for Implementation

### NPM Packages to Install

```bash
npm install @docusaurus/theme-mermaid
```

### External Tools Required (for content creation)

- **Python 3.10+**: For validating code snippets
- **ROS 2 Humble**: For testing runnable examples
- **Mermaid Live Editor**: For drafting diagrams (https://mermaid.live)
- **Prettier**: For Markdown formatting consistency

### Student Environment (documented in intro.md)

```bash
# Ubuntu 22.04 setup script
sudo apt update
sudo apt install python3-pip git
pip3 install rclpy
# ... (full setup in Quick Start guide)
```

---

## Implementation Phases Summary

### Phase 0: Preparation (1 file)
1. Install `@docusaurus/theme-mermaid`
2. Delete default tutorial folders
3. Update `docusaurus.config.ts` with Mermaid config

### Phase 1: Structure (5 directories + 5 _category_.json files)
1. Create `docs/module-01-ros2/` through `docs/module-05-capstone/`
2. Add `_category_.json` to each module directory
3. Update `sidebars.ts` with manual configuration

### Phase 2: Content - Module 1 (5 files)
1. Write `docs/intro.md` (textbook preface)
2. Write Module 1 chapters (01-05)
3. Add 6 Mermaid diagrams
4. Add 7 code examples

### Phase 3: Content - Module 2 (5 files)
1. Write Module 2 chapters (01-05)
2. Add 6 Mermaid diagrams
3. Add 5 code examples

### Phase 4: Content - Module 3 (5 files)
1. Write Module 3 chapters (01-05)
2. Add 8 Mermaid diagrams
3. Add 5 code examples

### Phase 5: Content - Module 4 (5 files)
1. Write Module 4 chapters (01-05)
2. Add 6 Mermaid diagrams
3. Add 5 code examples

### Phase 6: Content - Capstone (3 files)
1. Write Capstone chapters (01-03)
2. Add 4 Mermaid diagrams
3. Add 3 code examples

### Phase 7: Assets & Polish (3 files)
1. Create logo.svg
2. Update favicon.ico
3. Create social-card.jpg

### Phase 8: Validation & Deploy
1. Run build verification
2. Test all links and diagrams
3. Deploy to production

---

## Success Criteria Checklist

From the specification, the plan must ensure:

- [ ] **SC-001**: Beginners can navigate to Module 1 and see clear learning path
- [ ] **SC-002**: At least 30 Mermaid diagrams across all modules
- [ ] **SC-003**: 20+ runnable Python code examples with inline comments
- [ ] **SC-004**: Capstone project has clear requirements and rubric
- [ ] **SC-005**: Alternative paths documented for non-GPU learners
- [ ] **SC-006**: All code includes type hints and passes linting
- [ ] **SC-007**: Glossary terms linked throughout content (or inline tooltips)
- [ ] **SC-008**: Each module has self-assessment exercises

---

## Next Steps

1. **Approval Gate**: Review this plan with stakeholders
2. **Execute `/sp.tasks`**: Generate granular implementation tasks from this plan
3. **Start Implementation**: Begin with Phase 0-1 (structure setup)
4. **Iterative Review**: Complete one module, validate, then proceed to next

---

**Plan Status**: ✅ COMPLETE - Ready for `/sp.tasks` generation
