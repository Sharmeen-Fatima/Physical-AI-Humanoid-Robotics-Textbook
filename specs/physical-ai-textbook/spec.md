# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `physical-ai-textbook`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Create a comprehensive, spec-driven course on Physical AI & Humanoid Robotics covering ROS 2, Simulation, Isaac Sim, VLA, and a capstone project"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Complete Beginner Learning Path (Priority: P1)

A learner with basic Python knowledge but no robotics experience wants to understand how to build and control physical robots from zero to autonomous systems.

**Why this priority**: This is the primary audience and core value proposition. Without a clear path for beginners, the textbook fails its primary mission of democratizing Physical AI education.

**Independent Test**: A beginner can complete Module 1, write their first ROS 2 node, and see a simulated robot respond to their code within the first 3 hours of study.

**Acceptance Scenarios**:

1. **Given** a learner has Python installed, **When** they complete Module 1 Chapter 1, **Then** they understand ROS 2 architecture using biological analogies and can explain nodes, topics, and services
2. **Given** a learner has completed Module 1 exercises, **When** they write their first `rclpy` publisher node, **Then** they can send messages that control a simulated robot
3. **Given** a learner has finished Module 1, **When** they define a simple URDF file, **Then** they can visualize their custom robot geometry in RViz
4. **Given** a learner completes all 4 modules, **When** they attempt the capstone project, **Then** they can build an autonomous humanoid that responds to voice commands

---

### User Story 2 - Visual Learner Experience (Priority: P1)

A visual learner needs diagrams, illustrations, and code examples to understand abstract robotics concepts like coordinate transforms, message passing, and sensor fusion.

**Why this priority**: Robotics is inherently visual and spatial. Without diagrams, learners will struggle with coordinate frames, transforms, and architecture. This is critical for learning effectiveness.

**Independent Test**: Every technical concept (ROS 2 architecture, SLAM loop, VLA pipeline) has an accompanying Mermaid.js diagram that can be understood independently.

**Acceptance Scenarios**:

1. **Given** a learner is studying ROS 2 message flow, **When** they view the Mermaid.js diagram, **Then** they can trace a message from publisher through DDS to subscriber without reading text
2. **Given** a learner is learning SLAM, **When** they view the perception-planning-action loop diagram, **Then** they understand the data flow from LiDAR to map to path planner
3. **Given** a learner sees a VLA architecture diagram, **When** they study the visual, **Then** they can identify where Whisper, LLM, and ROS 2 action server components interact

---

### User Story 3 - Hands-On Practitioner Path (Priority: P2)

A developer with software engineering experience wants to quickly implement production-ready robotics code with best practices.

**Why this priority**: Practitioners need production-quality patterns, not toy examples. This audience will build real systems and needs spec-driven approaches.

**Independent Test**: Every module provides complete, runnable Python code snippets that follow ROS 2 conventions and can be integrated into production systems.

**Acceptance Scenarios**:

1. **Given** a developer completes a chapter, **When** they copy the provided code snippet, **Then** it runs without modification in their ROS 2 environment
2. **Given** a developer is implementing Nav2, **When** they follow the module's code examples, **Then** they get a production-ready configuration with error handling
3. **Given** a developer wants to test in simulation first, **When** they follow Module 2 guidance, **Then** they can validate their algorithm in Gazebo before deploying to hardware

---

### User Story 4 - Concept-to-Real Transfer (Priority: P2)

A robotics enthusiast wants to understand how to move from simulation (Isaac Sim, Gazebo, Unity) to real-world robots without getting stuck in the "sim-to-real gap."

**Why this priority**: The sim-to-real gap is the biggest blocker for learners. Addressing this explicitly creates unique value and prevents abandonment.

**Independent Test**: Module 3 provides specific techniques (domain randomization, sensor noise modeling, material property tuning) that can be tested by comparing simulated vs. real sensor data.

**Acceptance Scenarios**:

1. **Given** a learner has a working Gazebo simulation, **When** they apply domain randomization techniques, **Then** their model generalizes better to real-world conditions
2. **Given** a learner is using Isaac Sim, **When** they configure synthetic data generation, **Then** they can train perception models that work on real robot cameras
3. **Given** a learner has tuned PID controllers in simulation, **When** they deploy to hardware, **Then** they use the provided transfer checklist to adapt parameters systematically

---

### User Story 5 - Natural Language Robot Control (Priority: P3)

An AI enthusiast wants to build a robot that responds to natural language commands using modern LLMs (GPT-4, Gemini) integrated with ROS 2.

**Why this priority**: This is cutting-edge and exciting but builds on all prior modules. It's the "wow factor" that motivates learners but can only be achieved after foundational knowledge.

**Independent Test**: Module 4 provides a complete VLA pipeline that takes a voice command like "navigate to the kitchen" and executes ROS 2 navigation actions.

**Acceptance Scenarios**:

1. **Given** a learner has Module 4 VLA stack running, **When** they speak "move forward 2 meters", **Then** OpenAI Whisper transcribes it and the robot executes the motion
2. **Given** an LLM receives "go to the table and pick up the cup", **When** the VLA system parses the command, **Then** it generates a sequence of ROS 2 action calls (navigate, grasp)
3. **Given** a learner completes the capstone, **When** they integrate voice, vision, and navigation, **Then** they have an autonomous humanoid that can receive complex multi-step instructions

---

### Edge Cases

- **What happens when** a learner doesn't have NVIDIA GPU access for Isaac Sim?
  - *Solution*: Provide Gazebo-only alternative paths and cloud GPU guidance
- **What happens when** ROS 2 version mismatches occur between textbook and learner's system?
  - *Solution*: Explicitly document ROS 2 Humble as the target, provide version compatibility matrix
- **What happens when** a learner's Python environment has dependency conflicts?
  - *Solution*: Provide Docker containers and `requirements.txt` for each module
- **What happens when** simulation physics don't match real-world behavior?
  - *Solution*: Module 2 includes a troubleshooting guide for tuning friction, inertia, and damping parameters
- **What happens when** the LLM generates invalid ROS 2 action calls?
  - *Solution*: Module 4 includes validation layer and error recovery patterns

## Requirements *(mandatory)*

### Functional Requirements

#### Module 1: The Robotic Nervous System (ROS 2)

- **FR-001**: Textbook MUST explain ROS 2 architecture (nodes, topics, services, actions) using biological nervous system analogies
- **FR-002**: Textbook MUST provide runnable Python (`rclpy`) code examples for publishers, subscribers, and service servers
- **FR-003**: Textbook MUST include a Mermaid.js diagram showing ROS 2 message flow from node to DDS to node
- **FR-004**: Textbook MUST teach URDF syntax with a hands-on example of defining a simple 2-link robot arm
- **FR-005**: Textbook MUST explain coordinate frames (base_link, odom, map) with visual TF tree diagrams
- **FR-006**: Textbook MUST provide exercises where learners write a node that subscribes to sensor data and publishes control commands

#### Module 2: The Digital Twin (Simulation)

- **FR-007**: Textbook MUST cover Gazebo simulation with physics properties (gravity, friction, collision)
- **FR-008**: Textbook MUST provide a world file example with obstacles, lighting, and sensor plugins
- **FR-009**: Textbook MUST demonstrate Unity integration for high-fidelity human-robot interaction scenarios
- **FR-010**: Textbook MUST include Mermaid.js diagrams for sensor data flow (LiDAR point cloud → ROS topic → SLAM node)
- **FR-011**: Textbook MUST teach how to simulate RGB-D cameras, IMUs, and LiDAR with realistic noise models
- **FR-012**: Textbook MUST provide a troubleshooting guide for sim-to-real transfer issues (physics parameter tuning)

#### Module 3: The AI-Robot Brain (Isaac Sim & Nav2)

- **FR-013**: Textbook MUST introduce NVIDIA Isaac Sim with installation steps and system requirements
- **FR-014**: Textbook MUST demonstrate synthetic data generation for training perception models
- **FR-015**: Textbook MUST cover Nav2 architecture (costmaps, planners, controllers, behavior trees)
- **FR-016**: Textbook MUST provide Mermaid.js diagram of Nav2's planning pipeline
- **FR-017**: Textbook MUST include code examples for configuring Nav2 with custom costmap plugins
- **FR-018**: Textbook MUST teach SLAM using `slam_toolbox` with live mapping examples
- **FR-019**: Textbook MUST provide benchmarking guidance for comparing planners (DWB, TEB, MPPI)

#### Module 4: Vision-Language-Action (VLA)

- **FR-020**: Textbook MUST integrate OpenAI Whisper for speech-to-text transcription
- **FR-021**: Textbook MUST demonstrate LLM integration (GPT-4 or Gemini) for command parsing
- **FR-022**: Textbook MUST provide a Mermaid.js diagram of the VLA pipeline: Voice → Whisper → LLM → ROS 2 Action
- **FR-023**: Textbook MUST include code for a ROS 2 action server that receives parsed commands
- **FR-024**: Textbook MUST provide prompt engineering examples for LLM robot control
- **FR-025**: Textbook MUST include error handling for invalid or ambiguous commands

#### Capstone Project: The Autonomous Humanoid

- **FR-026**: Textbook MUST define a capstone project that integrates all 4 modules
- **FR-027**: Capstone MUST require voice command input, path planning (Nav2), and task execution in simulation
- **FR-028**: Textbook MUST provide acceptance criteria and rubric for self-assessment
- **FR-029**: Textbook MUST include starter code and a project template repository

#### Cross-Cutting Requirements

- **FR-030**: Every technical concept MUST have at least one Mermaid.js diagram
- **FR-031**: Every code snippet MUST be syntax-highlighted and include inline comments
- **FR-032**: Textbook MUST use a "co-learning" tone (teacher + AI collaboration)
- **FR-033**: Textbook MUST provide a "Quick Start" guide for environment setup (ROS 2, Gazebo, Python)
- **FR-034**: Textbook MUST include a glossary of robotics terminology
- **FR-035**: Textbook MUST link to external resources (ROS 2 docs, Isaac Sim tutorials, Nav2 guides)

### Key Entities *(include if feature involves data)*

- **Module**: A major learning unit (e.g., "Module 1: ROS 2"). Contains multiple chapters, exercises, and diagrams.
- **Chapter**: A subsection within a module covering a specific topic (e.g., "Chapter 1.2: Publishers and Subscribers").
- **Code Snippet**: A runnable Python example with inline comments, syntax highlighting, and explanation.
- **Mermaid Diagram**: A visual representation of architecture, data flow, or system components embedded in Markdown.
- **Exercise**: A hands-on task with acceptance criteria that learners complete to test understanding.
- **Glossary Entry**: A definition of a technical term (e.g., "URDF", "TF Tree", "Costmap") with links to related concepts.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A beginner with Python knowledge can complete Module 1 and run their first ROS 2 node within 3 hours
- **SC-002**: Every module has at least 5 Mermaid.js diagrams covering key architectural and data flow concepts
- **SC-003**: Every technical concept (80+ topics across 4 modules) has at least one runnable Python code example
- **SC-004**: Learners can complete the capstone project and demonstrate a voice-controlled autonomous robot in simulation
- **SC-005**: The textbook provides alternative paths for learners without NVIDIA GPUs (Gazebo-only track)
- **SC-006**: At least 90% of code snippets run without modification in a standard ROS 2 Humble + Python 3.10 environment
- **SC-007**: The textbook includes a comprehensive glossary with 100+ robotics terms linked throughout the content
- **SC-008**: Each module concludes with self-assessment exercises that learners can verify independently

## Content Structure *(detailed outline)*

### Module 1: The Robotic Nervous System (ROS 2)

#### Chapter 1.1: The Biological Analogy
- **Goal**: Introduce ROS 2 by comparing it to the nervous system
- **Mermaid Diagram**: Nervous system (brain, spinal cord, neurons) mapped to ROS 2 (master node, topics, messages)
- **Content**:
  - Why robots need middleware
  - The pub/sub pattern as neural signaling
  - Nodes as functional brain regions
- **Code Example**: None (conceptual introduction)

#### Chapter 1.2: Your First ROS 2 Node
- **Goal**: Write a minimal publisher and subscriber
- **Mermaid Diagram**: Node graph showing publisher → topic → subscriber
- **Content**:
  - Setting up a ROS 2 workspace
  - Creating a Python package
  - Writing a "talker" (publisher) node
  - Writing a "listener" (subscriber) node
- **Code Example**:
  ```python
  # talker.py - Publisher node
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class TalkerNode(Node):
      def __init__(self):
          super().__init__('talker')
          self.publisher = self.create_publisher(String, 'chatter', 10)
          self.timer = self.create_timer(1.0, self.publish_message)

      def publish_message(self):
          msg = String()
          msg.data = 'Hello from Physical AI'
          self.publisher.publish(msg)
          self.get_logger().info(f'Publishing: {msg.data}')

  def main():
      rclpy.init()
      node = TalkerNode()
      rclpy.spin(node)
      rclpy.shutdown()
  ```

#### Chapter 1.3: Services and Actions
- **Goal**: Understand request/response vs. long-running goals
- **Mermaid Diagram**: Service call flow vs. Action feedback loop
- **Content**:
  - When to use services (quick queries)
  - When to use actions (navigation, manipulation)
  - Writing a service server and client
- **Code Example**: Service server for "add two integers"

#### Chapter 1.4: URDF - The Robot's Body Blueprint
- **Goal**: Define robot geometry and kinematic chains
- **Mermaid Diagram**: URDF link-joint tree for a 2-link arm
- **Content**:
  - XML structure of URDF
  - Links (rigid bodies) and joints (connections)
  - Visualizing in RViz
- **Code Example**: URDF for a simple mobile robot with a camera

#### Chapter 1.5: Coordinate Frames and TF
- **Goal**: Master spatial relationships in robotics
- **Mermaid Diagram**: TF tree showing world → odom → base_link → camera_link
- **Content**:
  - Why coordinate frames matter
  - Broadcasting transforms
  - Listening to transforms
- **Code Example**: Python node that broadcasts a static transform

---

### Module 2: The Digital Twin (Simulation)

#### Chapter 2.1: Why Simulate First?
- **Goal**: Justify simulation before hardware
- **Mermaid Diagram**: Sim-to-real pipeline (design → test → deploy)
- **Content**:
  - Cost and safety benefits
  - Iteration speed
  - Synthetic data generation
- **Code Example**: None (conceptual)

#### Chapter 2.2: Gazebo Fundamentals
- **Goal**: Launch a simulated robot world
- **Mermaid Diagram**: Gazebo architecture (physics engine, sensor plugins, ROS bridge)
- **Content**:
  - Installing Gazebo Garden/Harmonic
  - Creating world files (.sdf)
  - Adding models from Gazebo Fuel
- **Code Example**: Launch file for Gazebo + RViz + robot spawn

#### Chapter 2.3: Physics and Sensors
- **Goal**: Configure realistic physics and sensor models
- **Mermaid Diagram**: Sensor data flow (LiDAR plugin → `/scan` topic → SLAM node)
- **Content**:
  - Tuning gravity, friction, damping
  - LiDAR, RGB-D, IMU sensor plugins
  - Adding noise models
- **Code Example**: SDF snippet for a GPU-accelerated LiDAR sensor

#### Chapter 2.4: Unity for High-Fidelity HRI
- **Goal**: Use Unity for photorealistic human-robot interaction
- **Mermaid Diagram**: Unity-ROS integration via ROS-TCP-Connector
- **Content**:
  - Setting up Unity + ROS 2
  - Importing robot models (URDF → Unity)
  - Real-time visualization
- **Code Example**: Unity C# script subscribing to ROS topic

#### Chapter 2.5: Sim-to-Real Transfer Checklist
- **Goal**: Provide actionable steps to bridge the gap
- **Content**:
  - Domain randomization techniques
  - Parameter tuning (PID gains, mass, inertia)
  - Validation metrics (compare sensor readings)
- **Code Example**: Python script for randomizing lighting and textures in Gazebo

---

### Module 3: The AI-Robot Brain (Isaac Sim & Nav2)

#### Chapter 3.1: NVIDIA Isaac Sim Introduction
- **Goal**: Set up Isaac Sim for synthetic data
- **Mermaid Diagram**: Isaac Sim architecture (Omniverse, RTX ray tracing, ROS 2 bridge)
- **Content**:
  - System requirements (NVIDIA GPU, drivers)
  - Installing Isaac Sim
  - Importing robots (Carter, Franka)
- **Code Example**: Python script to spawn a robot and capture RGB images

#### Chapter 3.2: Synthetic Data for Perception
- **Goal**: Generate labeled datasets for object detection
- **Mermaid Diagram**: Synthetic data pipeline (randomize scene → render → export labels)
- **Content**:
  - Replicator API for scene randomization
  - Bounding box and segmentation annotations
  - Training YOLOv8 on synthetic data
- **Code Example**: Python script using Replicator to generate 10,000 annotated images

#### Chapter 3.3: Nav2 Architecture
- **Goal**: Understand autonomous navigation stack
- **Mermaid Diagram**: Nav2 pipeline (sensor → costmap → planner → controller → cmd_vel)
- **Content**:
  - Global vs. local costmaps
  - Planners (NavFn, Smac, TEB)
  - Controllers (DWB, MPPI)
  - Behavior trees for task sequencing
- **Code Example**: Nav2 configuration YAML file with custom costmap layers

#### Chapter 3.4: SLAM with slam_toolbox
- **Goal**: Build maps from sensor data
- **Mermaid Diagram**: SLAM loop (LiDAR → scan matching → pose graph → map)
- **Content**:
  - Online vs. offline SLAM
  - Loop closure detection
  - Saving and loading maps
- **Code Example**: Launch file for `slam_toolbox` with cartographer comparison

#### Chapter 3.5: Path Planning Algorithms
- **Goal**: Compare planning approaches
- **Mermaid Diagram**: A* vs. RRT* vs. DWB comparison flowchart
- **Content**:
  - When to use global vs. local planners
  - Benchmarking metrics (time, smoothness, safety)
- **Code Example**: Python script to benchmark different Nav2 planners

---

### Module 4: Vision-Language-Action (VLA)

#### Chapter 4.1: The VLA Revolution
- **Goal**: Introduce natural language robot control
- **Mermaid Diagram**: VLA pipeline (Voice → ASR → LLM → Action Server → Robot)
- **Content**:
  - Why LLMs change robotics
  - Examples: RT-2, PaLM-E, Aloha
- **Code Example**: None (conceptual)

#### Chapter 4.2: Voice Capture with OpenAI Whisper
- **Goal**: Transcribe speech to text
- **Mermaid Diagram**: Audio → Whisper model → text output
- **Content**:
  - Installing Whisper
  - Microphone integration
  - Real-time vs. batch processing
- **Code Example**: Python script capturing audio and transcribing with Whisper

#### Chapter 4.3: LLM Command Parsing
- **Goal**: Use GPT-4/Gemini to interpret commands
- **Mermaid Diagram**: Text → LLM → structured action (JSON)
- **Content**:
  - Prompt engineering for robotics
  - Few-shot examples
  - Structured output (function calling)
- **Code Example**: Python script using OpenAI API to parse "go to the kitchen" into Nav2 goal

#### Chapter 4.4: ROS 2 Action Server Integration
- **Goal**: Execute LLM-generated commands
- **Mermaid Diagram**: Action client → action server → robot execution
- **Content**:
  - Creating custom action definitions
  - Action server with feedback
  - Error recovery
- **Code Example**: ROS 2 action server that receives navigation goals from LLM

#### Chapter 4.5: Safety and Validation
- **Goal**: Prevent harmful commands
- **Mermaid Diagram**: Validation layer (LLM output → safety check → execute/reject)
- **Content**:
  - Whitelisting allowed actions
  - Bounding box constraints
  - Human-in-the-loop confirmation
- **Code Example**: Python validator that checks if LLM command is safe

---

### Capstone Project: The Autonomous Humanoid

#### Project Overview
- **Goal**: Build a fully autonomous humanoid that responds to voice commands, navigates obstacles, and executes tasks in simulation
- **Mermaid Diagram**: Full system architecture (all modules integrated)

#### Requirements
1. Robot hears command via microphone
2. Whisper transcribes to text
3. LLM parses command into ROS 2 action sequence
4. Nav2 plans path in mapped environment
5. Robot executes task and reports completion

#### Starter Code
- Docker environment with all dependencies
- Template repository with placeholder nodes
- Example commands and expected behaviors

#### Rubric
- Voice recognition accuracy: 90%+
- Path planning success rate: 95%+
- Task completion in <2 minutes
- Code follows ROS 2 conventions
- Documentation and comments

---

## Visual Requirements

### Mermaid.js Diagrams (Minimum 30 across all modules)

**Module 1 (6 diagrams)**:
1. ROS 2 architecture (nodes, topics, DDS)
2. Nervous system analogy mapping
3. Publisher-subscriber message flow
4. Service call sequence
5. Action feedback loop
6. TF tree for mobile robot

**Module 2 (6 diagrams)**:
1. Sim-to-real pipeline
2. Gazebo architecture (physics, plugins, ROS)
3. Sensor data flow (LiDAR → topic → node)
4. Unity-ROS integration
5. Domain randomization workflow
6. Simulation validation checklist

**Module 3 (8 diagrams)**:
1. Isaac Sim architecture
2. Synthetic data generation pipeline
3. Nav2 full stack (sensors → costmap → planner → controller)
4. Global vs. local costmap layers
5. SLAM loop (scan → pose graph → map)
6. Behavior tree for task sequencing
7. Path planning algorithm comparison
8. Costmap inflation visualization

**Module 4 (6 diagrams)**:
1. VLA full pipeline (voice → action)
2. Whisper ASR workflow
3. LLM prompt → structured output
4. Action server lifecycle
5. Safety validation layer
6. Error recovery decision tree

**Capstone (4 diagrams)**:
1. Full system integration architecture
2. Data flow from voice to robot motion
3. State machine for task execution
4. Deployment checklist

---

## Tone and Style Guide

### Co-Learning Voice
- Use "we" and "let's" (e.g., "Let's build our first ROS 2 node")
- Frame the textbook as a journey: "Together, we'll master Physical AI"
- Acknowledge AI as a co-teacher: "With tools like ChatGPT and Claude, we can debug faster"

### Visionary but Practical
- Inspire with the future: "Imagine a world where robots serve coffee on voice command"
- Ground in reality: "But first, we need to understand coordinate frames"
- Celebrate incremental wins: "You just wrote your first publisher! That's a huge milestone."

### Accessibility
- Define jargon immediately: "URDF (Unified Robot Description Format) is an XML file that describes..."
- Provide analogies: "Think of a ROS topic like a radio channel - anyone can tune in"
- Use active voice: "The planner computes a path" (not "A path is computed by the planner")

---

## Technical Constraints

- **ROS 2 Version**: Humble (LTS) as primary target
- **Python Version**: 3.10+
- **Simulation**:
  - Gazebo Garden or Harmonic
  - Unity 2021.3+ (for HRI modules)
  - NVIDIA Isaac Sim 2023.1+ (requires RTX GPU)
- **Dependencies**:
  - OpenAI Whisper (or Hugging Face alternative)
  - OpenAI API / Google Gemini API for LLM
  - Nav2, slam_toolbox, RViz2
- **Hardware Assumptions**:
  - Primary path: NVIDIA GPU (for Isaac Sim)
  - Fallback path: CPU-only (Gazebo + Unity)
- **Code Quality**:
  - All snippets must pass `ruff` linting
  - Type hints for Python functions
  - Inline comments for clarity

---

## Out of Scope

- **Hardware-specific tutorials**: No Raspberry Pi, Jetson-specific instructions (focus on simulation-first)
- **Advanced control theory**: No detailed derivations of Kalman filters or PID math (conceptual only)
- **ROS 1**: Only ROS 2 covered
- **Non-humanoid robots**: Focus is humanoid, though examples use mobile robots for simplicity
- **Commercial deployments**: No production Kubernetes, edge deployment, or fleet management

---

## Dependencies and Assumptions

### Learner Prerequisites
- Comfortable with Python (functions, classes, async)
- Basic command-line usage (cd, ls, pip install)
- Familiarity with Git (clone, commit)
- Optional: Undergraduate-level physics (helpful but not required)

### System Requirements
- **Minimum**: Ubuntu 22.04, 8 GB RAM, 50 GB disk
- **Recommended**: Ubuntu 22.04, 16 GB RAM, NVIDIA RTX GPU, 100 GB disk
- **Fallback**: Docker on Windows/macOS (with performance warnings)

---

## Open Questions

- **Cloud GPU Options**: Should we provide Google Colab or AWS EC2 instructions for Isaac Sim?
- **LLM API Costs**: Should we estimate monthly costs for OpenAI API usage?
- **Video Content**: Will there be companion videos, or is this text-only?
- **Community**: Will learners have access to a Discord/forum for questions?
- **Certification**: Is there a completion certificate or assessment?

---

## Next Steps (for /sp.plan)

1. Define chapter-by-chapter Mermaid diagram specifications
2. Create code snippet templates with acceptance tests
3. Design exercise structure with auto-grading criteria
4. Plan Docusaurus content organization (sidebar, navigation)
5. Identify dependencies between modules (can Module 3 be done before Module 2?)
6. Create a "Quick Start" installation script for Ubuntu 22.04
