---
sidebar_position: 1
title: Voice Control with Whisper
---

# Voice Control: Speech-to-Text with OpenAI Whisper

**Learning Objectives**:
- Understand Automatic Speech Recognition (ASR)
- Set up OpenAI Whisper for voice capture
- Integrate speech-to-text with ROS 2
- Build a voice-controlled robot

**Prerequisites**: Module 3 complete, Python 3.10+, microphone access

**Estimated Time**: 1.5 hours

---

## The Vision-Language-Action (VLA) Revolution

Imagine telling your robot:
> "Go to the kitchen, pick up the cup, and bring it to me."

And it **just works**. This is the promise of **VLA**—robots that understand natural language and translate it to actions.

```mermaid
flowchart LR
    A[Voice Input] --> B[Whisper ASR]
    B --> C[Text Output]
    C --> D[LLM Reasoning]
    D --> E[ROS 2 Actions]
    E --> F[Robot Execution]

    style A fill:#b366ff
    style B fill:#00d9ff
    style D fill:#00d9ff
    style F fill:#b366ff
```

In this module, we start with **Step 1: Voice Capture using Whisper**.

---

## What is OpenAI Whisper?

**Whisper** is a state-of-the-art ASR model from OpenAI. It:
- Transcribes speech to text
- Supports 99 languages
- Works offline (after model download)
- Handles noisy environments well

:::tip Why Whisper?
Unlike cloud APIs (Google Speech, AWS Transcribe), Whisper runs locally—no internet required, no privacy concerns.
:::

---

## Installation

```bash
# Install Whisper
pip install openai-whisper

# Install audio processing library
pip install sounddevice numpy
```

Download models:
```bash
# Whisper downloads models automatically on first use
# Models: tiny, base, small, medium, large
# We'll use "base" for speed/accuracy balance
```

---

## Code Example: Capture and Transcribe

```python
# Example: Capture audio from microphone and transcribe with Whisper
# File: voice_capture.py

import whisper
import sounddevice as sd
import numpy as np
import wave

class VoiceCapture:
    """Captures voice and transcribes using Whisper."""

    def __init__(self, model_size: str = "base"):
        print(f"Loading Whisper model: {model_size}...")
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000  # Whisper expects 16kHz

    def record_audio(self, duration: int = 5) -> np.ndarray:
        """Record audio from microphone for specified duration."""
        print(f"Recording for {duration} seconds... Speak now!")
        audio = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()  # Wait until recording is finished
        print("Recording complete!")
        return audio.flatten()

    def transcribe(self, audio: np.ndarray) -> str:
        """Transcribe audio to text using Whisper."""
        print("Transcribing...")
        result = self.model.transcribe(audio, fp16=False)
        return result["text"].strip()

def main():
    # Initialize
    vc = VoiceCapture(model_size="base")

    # Record and transcribe
    audio = vc.record_audio(duration=5)
    text = vc.transcribe(audio)

    print(f"\n✅ You said: {text}")

if __name__ == "__main__":
    main()
```

**How to run**:
```bash
python3 voice_capture.py
# Speak into your microphone when prompted
# Example output: "✅ You said: move forward two meters"
```

---

## Integrating with ROS 2

Now let's publish the transcribed text to a ROS 2 topic.

### Code Example: ROS 2 Voice Node

```python
# Example: ROS 2 node that publishes voice commands
# File: voice_command_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np

class VoiceCommandNode(Node):
    """ROS 2 node that captures voice and publishes commands."""

    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher = self.create_publisher(String, '/voice_command', 10)
        self.model = whisper.load_model("base")
        self.sample_rate = 16000

        self.get_logger().info("Voice Command Node ready! Press Enter to record...")
        self.listen_loop()

    def listen_loop(self):
        """Continuously listen for voice commands."""
        while rclpy.ok():
            input("Press Enter to speak...")
            audio = self.record_audio(duration=5)
            text = self.transcribe(audio)

            # Publish to ROS topic
            msg = String()
            msg.data = text
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: "{text}"')

    def record_audio(self, duration: int) -> np.ndarray:
        self.get_logger().info(f'Recording for {duration}s...')
        audio = sd.rec(int(duration * self.sample_rate), samplerate=self.sample_rate, channels=1, dtype='float32')
        sd.wait()
        return audio.flatten()

    def transcribe(self, audio: np.ndarray) -> str:
        result = self.model.transcribe(audio, fp16=False)
        return result["text"].strip()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to test**:
```bash
# Terminal 1: Run the voice node
python3 voice_command_node.py

# Terminal 2: Echo the commands
ros2 topic echo /voice_command

# Speak: "navigate to the kitchen"
# Terminal 2 shows: data: 'navigate to the kitchen'
```

---

## Hands-On Exercise

**Challenge**: Create a voice-controlled robot that:
1. Listens for commands: "move forward", "turn left", "turn right", "stop"
2. Publishes appropriate Twist messages to `/cmd_vel`
3. Uses Whisper for speech recognition

**Acceptance Criteria**:
- [ ] Voice commands are transcribed accurately (>80%)
- [ ] Robot responds to spoken commands in simulation
- [ ] Node handles background noise gracefully

**Hints**:
- Use `geometry_msgs/Twist` for velocity commands
- Map keywords: "forward" → linear.x = 0.5
- Add a simple keyword parser (if "forward" in text: ...)

---

## Summary

**Key Takeaways**:
- Whisper provides state-of-the-art speech recognition
- Runs locally, no internet required
- Easy to integrate with ROS 2 via topic publishing
- Foundation for natural language robot control

**Next Steps**: In the [next chapter](./02-llm-control.md), we'll use LLMs (GPT-4, Gemini) to parse complex commands!

---

## Further Reading

- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [Whisper Model Card](https://github.com/openai/whisper/blob/main/model-card.md)
- [ROS 2 Audio Common](https://github.com/ros2/audio_common)
