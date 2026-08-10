import RPi.GPIO as GPIO
import time

BUZZER_PIN = 26

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

print(f"Starting buzzer test on GPIO {BUZZER_PIN} with 4kHz PWM...")
try:
    pwm = GPIO.PWM(BUZZER_PIN, 4000)
    pwm.start(50)
    time.sleep(2)
    pwm.stop()
except Exception as e:
    print(f"Error: {e}")
finally:
    GPIO.cleanup()
print("Done. Did you hear the buzzer?")
