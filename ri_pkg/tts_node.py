# tts_node.py

import pyttsx3

def speak(text):
    engine = pyttsx3.init()
    engine.setProperty('rate', 150)      # Adjust speech rate
    engine.setProperty('volume', 0.9)    # Volume: 0.0 to 1.0

    print(f"🔊 Speaking: {text}")
    engine.say(text)
    engine.runAndWait()

if __name__ == "__main__":
    user_input = input("Enter text to speak: ")
    speak(user_input)
