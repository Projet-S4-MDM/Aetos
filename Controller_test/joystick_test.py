import pygame
import sys

# Initialize Pygame and joystick
pygame.init()
pygame.joystick.init()

# Check if any joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    pygame.quit()
    sys.exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick Name: {joystick.get_name()}")
print(f"Number of Axes: {joystick.get_numaxes()}")
print(f"Number of Buttons: {joystick.get_numbuttons()}")

# Main loop
try:
    while True:
        # Process Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # Joystick button press
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")

            # Joystick button release
            if event.type == pygame.JOYBUTTONUP:
                print(f"Button {event.button} released")

            # Joystick axis motion
            if event.type == pygame.JOYAXISMOTION:
                print(f"Axis {event.axis} moved to {event.value:.2f}")

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()
    sys.exit()
