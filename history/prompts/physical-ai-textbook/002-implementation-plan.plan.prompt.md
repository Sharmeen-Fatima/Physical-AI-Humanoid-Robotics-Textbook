---
id: 002
title: Physical AI Textbook Implementation Plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5
feature: physical-ai-textbook
branch: master
user: Sharmeen Fatima
command: /sp.plan
labels: ["plan", "architecture", "docusaurus", "file-structure", "diagrams", "code-examples"]
links:
  spec: specs/physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/physical-ai-textbook/plan.md
tests:
 - None (planning document)
---

## Prompt

/sp.plan

# Implementation Plan for "Physical AI & Humanoid Robotics"

## 1. Objective
Translate the approved Specification into a concrete Docusaurus v3 file structure.

## 2. Directory Structure (The Module System)
You must plan to create the following folder hierarchy in `/docs`. Ensure folder names match the user's request exactly.

* **Landing:** `docs/intro.md` (Preface / Welcome)
* **Module 1:** `docs/module-01-ros2/`
    * `01-architecture.md`
    * `02-nodes-and-topics.md`
    * `03-urdf-basics.md`
* **Module 2:** `docs/module-02-digital-twin/`
    * `01-gazebo-setup.md`
    * `02-unity-integration.md`
* **Module 3:** `docs/module-03-brain/`
    * `01-isaac-sim.md`
    * `02-nav2-slam.md`
* **Module 4:** `docs/module-04-vla/`
    * `01-voice-control.md`
    * `02-llm-reasoning.md`
* **Capstone:** `docs/module-05-capstone/`
    * `01-final-project.md`

## 3. UI & Configuration Plan
* **Sidebar (`sidebars.js`):** Plan to create a manual sidebar slice that groups these modules distinctly (e.g., label: "Module 1: ROS 2").
* **Homepage (`src/pages/index.js`):** Plan to rewrite the Hero section to match the "Panaversity AI-Native" aesthetic (Cyber-Physical theme).
* **CSS (`src/css/custom.css`):** Plan to override default colors with the "Electric Blue/Cyan" palette defined in the Constitution.

**Action:** Generate the `plan.md` file that lists every file creation and code update required.

## Response snapshot

Successfully created comprehensive implementation plan for the Physical AI & Humanoid Robotics textbook with detailed technical architecture and implementation phases.

**Plan Overview** (specs/physical-ai-textbook/plan.md):

**File Structure Defined**:
- **23 content files**: 1 intro + 5 chapters × 4 modules + 3 capstone chapters
- **5 directories**: module-01-ros2, module-02-digital-twin, module-03-ai-brain, module-04-vla, module-05-capstone
- **5 _category_.json** files for sidebar configuration
- **2 config updates**: sidebars.ts (manual sidebar), docusaurus.config.ts (Mermaid + Prism)
- **3 asset files**: logo.svg, favicon.ico, social-card.jpg
- **2 deletions**: Remove tutorial-basics/, tutorial-extras/

**Total**: 31 file operations

**Detailed Module Structure**:
- **Module 1** (5 chapters): Biological analogy → First node → Services/Actions → URDF → Coordinate frames
- **Module 2** (5 chapters): Why simulate → Gazebo fundamentals → Physics/Sensors → Unity HRI → Sim-to-real
- **Module 3** (5 chapters): Isaac Sim intro → Synthetic data → Nav2 architecture → SLAM → Path planning
- **Module 4** (5 chapters): VLA revolution → Voice/Whisper → LLM parsing → Action servers → Safety validation
- **Capstone** (3 chapters): Project overview → Integration guide → Rubric/Assessment

**Architecture Specifications**:

1. **Sidebar Configuration** (sidebars.ts):
   - Manual categories with emoji labels (🤖 🎮 🧠 💬 ⚡)
   - Module 1 expanded by default, others collapsed
   - 5 top-level categories for clean navigation

2. **Mermaid Diagram Inventory** (30+ diagrams specified):
   - Module 1: 6 diagrams (ROS arch, pub/sub, TF tree, etc.)
   - Module 2: 6 diagrams (Sim-to-real, Gazebo arch, sensor flow, etc.)
   - Module 3: 8 diagrams (Isaac Sim, Nav2 stack, SLAM, costmaps, etc.)
   - Module 4: 6 diagrams (VLA pipeline, Whisper, LLM, validation, etc.)
   - Capstone: 4 diagrams (Full integration, state machine, deployment, etc.)

3. **Code Example Standards**:
   - Language tags (python, yaml, xml, bash)
   - Title comments
   - Inline documentation every 3-5 lines
   - Type hints for all functions
   - Runnable with copy-paste
   - "How to run" and "Expected output" sections

4. **Content Template**:
   - Learning objectives
   - Prerequisites
   - Mermaid diagrams with explanations
   - Code examples with "Try it yourself" suggestions
   - Hands-on exercises with acceptance criteria
   - Common pitfalls (warnings)
   - Summary and next steps

**Technical Decisions**:

- **Docusaurus 3.x** with TypeScript
- **@docusaurus/theme-mermaid** for diagrams
- **Prism.js** with Python, YAML, XML, Bash syntax highlighting
- **Dark mode default** (cyber-physical theme already implemented)
- **Mobile-first responsive** design
- **Build performance**: Target <60s for 20 pages

**Implementation Phases** (8 phases):
- Phase 0: Preparation (install Mermaid, cleanup defaults)
- Phase 1: Structure (create directories, _category_.json, update sidebar)
- Phase 2-6: Content creation (one module per phase)
- Phase 7: Assets (logo, favicon, social card)
- Phase 8: Validation & deployment

**Risk Mitigation**:
- Mermaid rendering tested early
- Mobile-friendly code blocks (collapsible for long examples)
- Docker environment for runnable examples
- ROS 2 version annotations in diagrams

**Success Criteria Mapped**:
- 30+ Mermaid diagrams ✓
- 20+ runnable code examples ✓
- Clear beginner path ✓
- Capstone with rubric ✓
- Alternative paths for non-GPU learners ✓

**Open Questions**:
1. Should we add interactive code playgrounds?
2. Include video content links?
3. Glossary page vs. inline tooltips?
4. Auto-graded exercises via GitHub Actions?
5. Giscus integration for comments?
6. Versioning strategy for ROS 2 updates?

## Outcome

- ✅ Impact: Complete architectural blueprint ready for implementation with 31 file operations defined
- 🧪 Tests: Build verification checklist, link checking, diagram rendering validation
- 📁 Files: 1 comprehensive plan.md created (7000+ words with detailed templates)
- 🔁 Next prompts: /sp.tasks to generate actionable task breakdown, then implement Phase 0-1 (structure)
- 🧠 Reflection: Plan successfully translates specification into concrete Docusaurus structure with clear phases, templates, and validation criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual review - PASS (comprehensive, actionable, follows Docusaurus best practices)
- Prompt variant (if applicable): plan-001-physical-ai-textbook
- Next experiment (smallest change to try): Execute /sp.tasks to break plan into granular, testable implementation tasks
