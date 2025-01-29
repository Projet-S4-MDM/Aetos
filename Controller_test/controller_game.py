import pygame
import sys

# Initialize Pygame
pygame.init()

# Screen setup
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Controller Game")

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)

# Square setup
square_size = 50
square_x, square_y = WIDTH // 2, HEIGHT // 2
speed = 5

# Initialize the controller
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No controller detected!")
    pygame.quit()
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Controller connected: {joystick.get_name()}")

# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill(WHITE)

    # Get joystick axes
    x_axis = joystick.get_axis(0)  # Left joystick horizontal
    y_axis = joystick.get_axis(1)  # Left joystick vertical

    # Update square position based on joystick input
    square_x += int(x_axis * speed)
    square_y += int(y_axis * speed)

    # Keep the square within screen bounds
    square_x = max(0, min(WIDTH - square_size, square_x))
    square_y = max(0, min(HEIGHT - square_size, square_y))

    # Draw the square
    pygame.draw.rect(screen, BLUE, (square_x, square_y, square_size, square_size))

    # Update the screen
    pygame.display.flip()

# Quit Pygame
pygame.quit()
