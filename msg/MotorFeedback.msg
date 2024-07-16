# This message contains status of the motor.

float32 position  # Motor position (degrees)
float32 velocity  # Motor radial velocity (degrees/sec)
float32 current   # Motor current drawn (amperes)
int8 temperature  # Motor hardware temperature (celcius)

uint8 motor_fault # Motor fault type
                  # 0: None
                  # 1: Overtemperature
                  # 2: Overcurrent
                  # 3: Overvoltage
                  # 4: Undervoltage
                  # 5: Encoder Error
                  # 6: Hardware Error