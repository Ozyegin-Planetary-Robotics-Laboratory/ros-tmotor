# This message contains posvelacc commands for an actuator.

uint8 type            # Type of command
                      # 0: position velocity acceleration
                      # 1: position
                      # 2: velocity

float32 position      # Position command
float32 velocity      # Velocity command
float32 acceleration  # Acceleration command
