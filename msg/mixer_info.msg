uint32 id
float32[3] position
float32[3] axis         # Z down, Y right, X front
float32 thrust_constant # Ct in N/command
float32 torque_constant # Cm in Nm/command

uint8 direction
uint8 DIRECTION_CLOCKWISE = 0
uint8 DIRECTION_COUNTERCLOCKWISE = 1

