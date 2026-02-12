# FRC Elevator Development Notes

## REBUILT Challenge - Elevator Design Considerations

### Overview
This document contains development notes, design decisions, and implementation considerations for the elevator subsystem in the REBUILT challenge.

### Mechanical Considerations
- **Travel Distance**: Determine maximum and minimum elevator positions based on game requirements
- **Motor Selection**: Typically use NEO, Falcon 500, or CIM motors for elevator mechanisms
- **Gearing**: Calculate gear ratio for optimal speed vs. torque balance
- **Safety**: Implement soft limits and hard stops to prevent damage
- **Cable/Chain Management**: Ensure proper tension and routing

### Electrical Setup
- **Motor Controllers**: Spark MAX (for NEO), Talon FX (for Falcon), or other supported controllers
- **Encoders**: Use built-in motor encoders or external encoders for position feedback
- **Limit Switches**: Optional physical limit switches for safety
- **Power Distribution**: Ensure adequate power delivery for motor(s)

### Software Architecture

#### Position Control
- Use PID control for accurate position hold and movement
- Implement feedforward for gravity compensation
- Set acceleration and velocity constraints for smooth motion

#### Safety Features
- Soft limits (upper and lower bounds in software)
- Current limiting to prevent motor burnout
- Emergency stop capability
- Position validation before movement

#### Control Modes
1. **Manual Control**: Operator direct control with joystick
2. **Position Presets**: Predefined positions for game elements
3. **Hold Position**: Maintain current position against gravity

### PID Tuning Guidelines
1. Start with P gain only
2. Increase P until oscillation occurs, then reduce by 50%
3. Add D to dampen oscillations if needed
4. Add I only if there's steady-state error (usually not needed with feedforward)
5. Implement feedforward (kG) to counteract gravity

### Common Preset Positions (Examples)
- **STOWED**: Elevator fully retracted (0 inches/rotations)
- **LOW**: Low scoring position
- **MID**: Medium scoring position
- **HIGH**: High scoring position
- **INTAKE**: Position for picking up game elements

### Testing Checklist
- [ ] Verify encoder direction matches motor direction
- [ ] Test soft limits prevent over-extension
- [ ] Confirm PID holds position under load
- [ ] Check current draw is within acceptable limits
- [ ] Test all preset positions
- [ ] Verify emergency stop functionality
- [ ] Test integration with RobotContainer commands

### Known Issues / TODO
- Fine-tune PID constants during testing
- Calibrate exact preset positions based on field measurements
- Implement current limiting thresholds
- Add telemetry for SmartDashboard/Shuffleboard

### Resources
- WPILib Documentation: https://docs.wpilib.org/
- Chief Delphi Forums: https://www.chiefdelphi.com/
- Team 254 Code Examples: https://github.com/Team254
- FRC Control System Documentation: https://docs.wpilib.org/en/stable/docs/controls-overviews/control-system-hardware.html

### Version History
- v1.0: Initial elevator subsystem creation
- Future: Add profiled PID, motion magic, or other advanced control methods
