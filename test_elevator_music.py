#!/usr/bin/env python3
"""
Test script for elevator music functionality
"""

import time
import pygame
import math
import numpy as np

def play_elevator_music():
    """Play mild elevator music using pygame."""
    try:
        # Initialize pygame mixer
        pygame.mixer.init()
        
        # Load and play the elevator music file
        music_file = "Elevator Music - aeiouFU.mp3"
        
        # Load the music file
        pygame.mixer.music.load(music_file)
        
        # Set volume to mild level (0.3 = 30% volume)
        pygame.mixer.music.set_volume(0.3)
        
        # Play the music (loops continuously)
        pygame.mixer.music.play(-1)  # -1 means loop indefinitely
        
        print("🎵 Playing elevator music...")
        
    except Exception as e:
        print(f"Elevator music error: {e}")

def test_elevator_music():
    """Test elevator music functionality."""
    print("Testing elevator music...")
    print("Make sure your speakers are on!")
    print("Press Ctrl+C to stop the music test")
    
    # Play elevator music
    play_elevator_music()
    
    try:
        # Keep playing for 10 seconds or until interrupted
        time.sleep(10)
    except KeyboardInterrupt:
        print("\nStopping music test...")
    finally:
        # Stop the music
        pygame.mixer.music.stop()
        print("✅ Elevator music test completed!")

if __name__ == "__main__":
    test_elevator_music() 