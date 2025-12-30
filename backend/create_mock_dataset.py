#!/usr/bin/env python3
"""
Mock Dataset Creator for RAG Chatbot Testing

Creates realistic book content about Physical AI & Humanoid Robotics
for testing the complete RAG pipeline without requiring the actual book website.

Usage:
    python create_mock_dataset.py
"""

import sys
from pathlib import Path
from datetime import datetime
from uuid import uuid4

# Add backend to path for src imports
backend_path = Path(__file__).parent
sys.path.insert(0, str(backend_path))

from src.config import get_settings
from src.embedding import CohereEmbedder
from src.storage import qdrant_client
from src.storage.schemas import TextChunk
from src.utils.logger import setup_logging, get_logger

logger = get_logger(__name__)


# Mock book content - realistic chapters about Physical AI & Humanoid Robotics
MOCK_BOOK_CONTENT = [
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter1",
        "section": "Chapter 1: Introduction to Physical AI",
        "content": """
Physical AI represents the convergence of artificial intelligence and robotics, enabling machines
to interact with and manipulate the physical world. Unlike traditional AI systems that operate
purely in digital spaces, Physical AI systems must understand and respond to real-world physics,
materials, and dynamics.

The key components of a Physical AI system include sensors for perception, actuators for movement,
control systems for coordination, and intelligence layers for decision-making. These components
work together in a tightly integrated feedback loop, continuously sensing the environment and
adjusting actions in real-time.

Modern Physical AI systems leverage deep learning for vision, reinforcement learning for motion
planning, and transformer models for language understanding. This multi-modal approach enables
robots to perceive their surroundings, plan complex actions, and interact naturally with humans
through language and gesture.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter2",
        "section": "Chapter 2: Humanoid Robot Architecture",
        "content": """
Humanoid robots are designed to replicate human form and function, with bipedal locomotion,
articulated arms, and anthropomorphic features. The architecture of a humanoid robot consists
of several interconnected subsystems: mechanical structure, sensory systems, control hierarchy,
and cognitive processing.

The mechanical structure includes the skeletal framework, joints with varying degrees of freedom,
and actuators (motors or hydraulics) that provide movement. Modern humanoids use lightweight
materials like carbon fiber and advanced alloys to optimize strength-to-weight ratios while
maintaining structural integrity.

Sensory systems provide the robot with awareness of its environment and its own body state.
This includes visual sensors (cameras, depth sensors), tactile sensors in hands and feet,
inertial measurement units (IMUs) for balance, and joint encoders for proprioception. The fusion
of these sensor modalities enables robust perception even in challenging conditions.

The control hierarchy operates at multiple levels: low-level motor control maintains precise
joint positions, mid-level controllers coordinate limb movements, and high-level planners make
strategic decisions about tasks and navigation. This hierarchical approach enables both reactive
responses and deliberative planning.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter3",
        "section": "Chapter 3: Sensors and Perception",
        "content": """
Sensor systems are the eyes and ears of a humanoid robot, providing critical information about
the environment and the robot's own state. Vision sensors include RGB cameras for color perception,
depth cameras (stereo or time-of-flight) for 3D understanding, and sometimes LiDAR for precise
distance measurements.

Tactile sensors embedded in robotic hands enable force feedback and object manipulation. These
can range from simple contact switches to advanced multi-axis force/torque sensors that measure
pressure distribution across fingertips. Proprioceptive sensors, including joint encoders and
IMUs, provide awareness of body configuration and movement.

The perception pipeline processes raw sensor data through multiple stages: filtering and noise
reduction, feature extraction, object detection and segmentation, and semantic understanding.
Modern systems employ deep neural networks for vision tasks, achieving human-level performance
in object recognition and scene understanding.

Sensor fusion combines data from multiple modalities to create a coherent world model. Kalman
filters and particle filters are common techniques for fusing noisy sensor measurements over time,
while neural network approaches can learn optimal fusion strategies directly from data.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter4",
        "section": "Chapter 4: Actuators and Movement",
        "content": """
Actuators are the muscles of a humanoid robot, converting electrical, hydraulic, or pneumatic
energy into mechanical motion. Electric motors are the most common actuators, offering precise
control and efficiency. Servo motors with feedback loops maintain exact positions, while brushless
DC motors provide high torque and reliability.

Hydraulic actuators deliver exceptional power density, making them suitable for large humanoids
that need to carry heavy loads or perform forceful tasks. However, they require pumps, valves,
and fluid management systems that add complexity. Pneumatic actuators offer compliance and safety
but typically have less precise control.

Series elastic actuators (SEAs) incorporate springs between the motor and output, providing
inherent compliance that makes robots safer for human interaction. This elasticity also enables
energy storage and more natural, dynamic movements. Variable stiffness actuators extend this
concept by allowing real-time adjustment of mechanical impedance.

Gait generation for bipedal walking involves complex coordination of multiple joints while
maintaining balance. The Zero Moment Point (ZMP) criterion is a fundamental principle, ensuring
that the robot's center of pressure remains within the support polygon. Modern approaches use
model predictive control (MPC) to optimize footstep sequences and trajectories.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter5",
        "section": "Chapter 5: Machine Learning for Robotics",
        "content": """
Machine learning has transformed robotics by enabling systems to learn from experience rather
than relying solely on hand-crafted rules. Supervised learning trains models on labeled datasets,
commonly used for perception tasks like object detection and semantic segmentation. Convolutional
neural networks (CNNs) excel at visual recognition, while transformers have shown remarkable
performance on multi-modal tasks.

Reinforcement learning (RL) enables robots to learn complex behaviors through trial and error.
The robot receives rewards for desired outcomes and learns policies that maximize cumulative
reward. Deep reinforcement learning combines neural networks with RL algorithms like PPO
(Proximal Policy Optimization) or SAC (Soft Actor-Critic), enabling learning of high-dimensional
control policies.

Imitation learning allows robots to learn from human demonstrations. Behavioral cloning directly
mimics observed actions, while inverse reinforcement learning infers the underlying reward
function from demonstrations. These approaches can dramatically reduce the amount of trial-and-error
exploration needed.

Sim-to-real transfer addresses the challenge of training in simulation and deploying on real
robots. Domain randomization adds variability to simulated environments, making learned policies
more robust to real-world variations. System identification techniques help align simulation
parameters with physical robot characteristics.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter6",
        "section": "Chapter 6: Navigation and Planning",
        "content": """
Navigation enables a robot to move purposefully through its environment, requiring both perception
of obstacles and planning of collision-free paths. Simultaneous Localization and Mapping (SLAM)
algorithms build maps while tracking the robot's position, using techniques like particle filters
or graph optimization.

Path planning algorithms find routes from start to goal positions. Classical approaches include
A* search on occupancy grids and probabilistic roadmaps (PRMs) for high-dimensional configuration
spaces. Rapidly-exploring Random Trees (RRTs) efficiently explore complex environments by
randomly sampling and building tree structures.

Motion planning for manipulation involves coordinating multiple joints while avoiding obstacles
and respecting kinematic constraints. Trajectory optimization formulates motion planning as an
optimization problem, finding smooth, dynamically feasible paths that minimize energy or time.

Behavior trees provide a structured framework for task planning, organizing skills into
hierarchical compositions. Each node in the tree represents a behavior or condition, with
execution flowing based on success/failure states. This modularity enables complex task
composition from reusable primitives.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter7",
        "section": "Chapter 7: Human-Robot Interaction",
        "content": """
Human-robot interaction (HRI) encompasses the methods by which humans and robots communicate
and collaborate. Natural language processing enables robots to understand speech commands and
respond verbally. Modern systems use large language models like GPT for flexible dialogue,
combined with speech recognition (Whisper) and text-to-speech systems.

Gesture recognition allows intuitive non-verbal communication. Vision-based systems track hand
movements, body pose, and facial expressions using deep learning models. These signals can
indicate intent, emotion, or direct commands without requiring explicit verbal input.

Safety is paramount in HRI, especially for physical contact. Collaborative robots (cobots)
incorporate force limiting, collision detection, and compliant control to ensure safe operation
near humans. Emergency stops and workspace monitoring prevent dangerous situations.

Social robotics explores how robots can engage in social behaviors and build relationships with
humans. This includes appropriate gaze behavior, emotional expression through facial displays or
vocalizations, and adherence to social norms. Affective computing enables robots to recognize
and respond to human emotions.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter8",
        "section": "Chapter 8: Manipulation and Grasping",
        "content": """
Robotic manipulation involves controlled interaction with objects, from picking and placing to
assembly and tool use. Grasp planning determines how to position fingers or gripper elements to
securely hold an object. Analytical approaches model contact mechanics and force closure, while
learning-based methods train on large datasets of successful grasps.

End-effectors (grippers, hands) vary in complexity. Parallel jaw grippers are simple and reliable
for industrial tasks. Multi-fingered hands with articulated digits enable dexterous manipulation
but require sophisticated control. Soft grippers using compliant materials can adapt to object
shapes and handle fragile items safely.

Force control regulates the amount of force applied during contact. Impedance control modulates
the robot's mechanical impedance, making it respond compliantly to external forces. This is
essential for assembly tasks, polishing, or any situation requiring controlled contact forces.

Manipulation primitives decompose complex tasks into reusable skills: reaching, grasping,
transporting, placing, and releasing. Vision-guided manipulation uses visual servoing to continuously
adjust motions based on visual feedback, enabling precise alignment even with position uncertainties.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter9",
        "section": "Chapter 9: Control Systems and Stability",
        "content": """
Control theory provides the mathematical foundation for robot motion and stability. PID
(Proportional-Integral-Derivative) controllers are ubiquitous for basic motor control, adjusting
outputs based on error, accumulated error, and rate of change. Proper tuning of PID gains is
critical for stable, responsive performance.

State-space controllers model system dynamics as differential equations and use full state
feedback for control. Linear Quadratic Regulators (LQR) optimize control inputs to minimize
a cost function balancing tracking error and control effort. State estimation from Kalman
filters provides optimal state estimates from noisy measurements.

Model Predictive Control (MPC) optimizes a sequence of future control actions over a prediction
horizon, accounting for constraints and predicted disturbances. This enables anticipatory control
that's particularly valuable for walking robots, where future foot placements affect current
balance.

Robust control techniques ensure stability despite model uncertainties and disturbances.
H-infinity control minimizes worst-case performance degradation, while adaptive control adjusts
controller parameters online to accommodate changing dynamics. Lyapunov stability analysis
provides rigorous guarantees of convergence.
"""
    },
    {
        "url": "https://physical-ai-humanoid-robotics-textb-pied.vercel.app/book/chapter10",
        "section": "Chapter 10: Future of Physical AI",
        "content": """
The future of Physical AI promises increasingly capable, versatile robots that seamlessly integrate
into human environments. Foundation models for robotics, trained on massive multi-modal datasets,
will enable general-purpose robots that can quickly adapt to new tasks through few-shot learning
or natural language instruction.

Embodied AI emphasizes the importance of physical experience in developing intelligence. Just as
humans learn through interaction with the world, future AI systems will develop understanding
through robotic embodiment. This grounds abstract concepts in physical reality and enables
common-sense reasoning about objects, forces, and causality.

Soft robotics represents a paradigm shift from rigid mechanisms to compliant, adaptable structures.
Using materials like silicone, shape-memory alloys, and electroactive polymers, soft robots can
safely operate in unstructured environments and handle delicate objects. Biological inspiration
drives innovations in locomotion, manipulation, and sensing.

Swarm robotics explores collective behaviors emerging from simple individual rules. Large numbers
of small robots can accomplish tasks impossible for single agents, with applications in search
and rescue, environmental monitoring, and construction. Decentralized control makes swarms
resilient to individual failures.

Ethical considerations become increasingly important as robots gain capabilities. Questions of
autonomy, accountability, privacy, and economic impact require thoughtful frameworks. Human-centered
design ensures that Physical AI serves human needs and values, augmenting rather than replacing
human capabilities.
"""
    }
]


def create_mock_chunks(content_items):
    """Create mock text chunks from content."""
    chunks = []

    for item in content_items:
        # Split content into paragraphs
        paragraphs = [p.strip() for p in item["content"].strip().split('\n\n') if p.strip()]

        # Each paragraph becomes a chunk
        for idx, paragraph in enumerate(paragraphs):
            chunk = TextChunk(
                chunk_id=str(uuid4()),  # Generate proper UUID for Qdrant
                text=paragraph,
                chunk_index=idx,
                parent_url=item["url"],
                section_title=item["section"],
                char_start=item["content"].index(paragraph),
                char_end=item["content"].index(paragraph) + len(paragraph),
                overlap_start=0,
                overlap_end=0,
            )
            chunks.append(chunk)

    return chunks


def main():
    """Main entry point for mock dataset creation."""
    # Setup
    settings = get_settings()
    setup_logging(log_level=settings.log_level, log_file="logs/mock_dataset.log")

    logger.info("Starting mock dataset creation")
    print("\n" + "="*70)
    print("  MOCK DATASET CREATOR")
    print("="*70)
    print("\nCreating realistic Physical AI & Humanoid Robotics content")
    print("for testing the complete RAG pipeline.\n")

    # Step 1: Create chunks
    print("[1/4] Creating text chunks from mock content...")
    chunks = create_mock_chunks(MOCK_BOOK_CONTENT)
    print(f"  [OK] Created {len(chunks)} chunks from {len(MOCK_BOOK_CONTENT)} chapters\n")
    logger.info(f"Created {len(chunks)} chunks")

    # Show sample
    print("  Sample chunks:")
    for chunk in chunks[:3]:
        print(f"    - {chunk.section_title}")
        print(f"      {chunk.text[:80]}...")
    print()

    # Step 2: Generate embeddings
    print("[2/4] Generating embeddings with Cohere...")
    embedder = CohereEmbedder()

    chunk_texts = [chunk.text for chunk in chunks]
    print(f"  Processing {len(chunk_texts)} chunks...")

    try:
        embeddings = embedder.generate_embeddings_batch(
            texts=chunk_texts,
            input_type="search_document"
        )
        print(f"  [OK] Generated {len(embeddings)} embeddings (dim={len(embeddings[0])})\n")
        logger.info(f"Generated {len(embeddings)} embeddings")
    except Exception as e:
        print(f"  [FAIL] Embedding generation failed: {e}")
        logger.error(f"Embedding failed: {e}", exc_info=True)
        return 1

    # Step 3: Initialize Qdrant
    print("[3/4] Connecting to Qdrant...")
    qdrant = qdrant_client.QdrantClient()

    # Ensure collection exists
    print(f"  Ensuring collection '{qdrant.collection_name}' exists...")
    qdrant.create_collection(recreate=False)
    print(f"  [OK] Collection ready\n")

    # Step 4: Store in Qdrant
    print("[4/4] Storing chunks in Qdrant...")

    try:
        # Upsert chunks and embeddings
        num_upserted = qdrant.upsert_chunks(chunks, embeddings)

        if num_upserted > 0:
            print(f"  [OK] Successfully stored {num_upserted} chunks\n")
            logger.info(f"Stored {num_upserted} chunks in Qdrant")
        else:
            print(f"  [WARN] Upsert may have had issues - check logs\n")

    except Exception as e:
        print(f"  [FAIL] Qdrant storage failed: {e}")
        logger.error(f"Qdrant upsert failed: {e}", exc_info=True)
        return 1

    # Summary
    print("="*70)
    print("  MOCK DATASET CREATED SUCCESSFULLY")
    print("="*70)
    print(f"\nStatistics:")
    print(f"  Chapters: {len(MOCK_BOOK_CONTENT)}")
    print(f"  Chunks created: {len(chunks)}")
    print(f"  Embeddings generated: {len(embeddings)}")
    print(f"  Vector dimension: {len(embeddings[0])}")
    print(f"  Collection: {qdrant.collection_name}")
    print(f"  Source: Mock dataset (Physical AI & Humanoid Robotics)")

    print("\n" + "="*70)
    print("\nSample Queries to Test:")
    print("  - What are the main components of a humanoid robot?")
    print("  - How do sensors work in Physical AI systems?")
    print("  - Explain machine learning for robotics")
    print("  - What is the role of actuators in robot movement?")
    print("  - How do robots interact with humans?")

    print("\nNext Steps:")
    print("  1. Run: python test_chatbot_simple.py")
    print("     (Should now PASS all 4 tests)")
    print("\n  2. Run: python scripts/chatbot.py")
    print("     (Try the sample queries above)")
    print("\n" + "="*70 + "\n")

    logger.info("Mock dataset creation completed successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
