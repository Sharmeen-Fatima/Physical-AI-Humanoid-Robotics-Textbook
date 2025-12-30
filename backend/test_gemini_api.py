#!/usr/bin/env python3
"""Quick test to check available Gemini models."""

import google.generativeai as genai
from src.config import get_settings

settings = get_settings()

try:
    # Configure API
    genai.configure(api_key=settings.gemini_api_key)

    print("Fetching available models...\n")

    # List models
    for model in genai.list_models():
        if 'generateContent' in model.supported_generation_methods:
            print(f"Model: {model.name}")
            print(f"  Display name: {model.display_name}")
            print(f"  Description: {model.description[:80]}...")
            print()

except Exception as e:
    print(f"Error: {e}")
    print("\nThis might indicate:")
    print("  1. Invalid API key")
    print("  2. API key not enabled for Gemini API")
    print("  3. Billing not set up")
    print("  4. Region restrictions")
