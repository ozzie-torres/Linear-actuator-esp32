(FluidNC two-axis commissioning test - no spindle or tool output)
(Requires a completed homing cycle before execution)
(Uses machine coordinates and stays inside X5..15 mm and Y5..15 mm)

G21
G90
G94

(Move 5 mm away from both minimum switches)
G53 G0 X5 Y5

(Trace a 10 mm square at 300 mm/min)
G53 G1 X15 Y5 F300
G53 G1 X15 Y15 F300
G53 G1 X5 Y15 F300
G53 G1 X5 Y5 F300

(Return to the homed machine origin)
G53 G0 X0 Y0

M2
