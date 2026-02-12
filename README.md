# FRC Elevator Subsystem - REBUILT Challenge

A comprehensive elevator subsystem implementation for FRC robots, specifically designed for the REBUILT challenge. This repository contains a complete, production-ready elevator control system using WPILib Command-Based programming.

## 📁 Repository Contents

- **`Elevator.java`** - Main elevator subsystem with PID control, safety features, and telemetry
- **`RobotContainer.java`** - Robot configuration with controller bindings and command setup
- **`notes.md`** - Development notes, design considerations, and implementation guidelines
- **`README.md`** - This file - complete setup and usage instructions

## 🚀 Quick Start Guide

### Prerequisites

1. **WPILib Installation** (2024 or later)
   - Download from: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
   - Includes VS Code, Java JDK, and all FRC libraries

2. **Hardware Requirements**
   - REV Robotics NEO motor (or adapt for Falcon 500/CIM)
   - REV Robotics Spark MAX motor controller
   - Encoder (built-in to NEO motors)
   - RoboRIO 2.0 or compatible controller
   - Xbox controllers (driver and operator)

3. **Software Requirements**
   - FRC Driver Station
   - Phoenix Tuner (for CAN device configuration)
   - REV Hardware Client (for Spark MAX configuration)

### Installation Steps

1. **Clone or copy files to your robot project**
   ```bash
   # Option 1: Clone this repository
   git clone https://github.com/DRIFT-4091/Elevator.git
   
   # Option 2: Copy files manually
   # Copy Elevator.java to src/main/java/frc/robot/subsystems/
   # Copy RobotContainer.java to src/main/java/frc/robot/
   ```

2. **Configure CAN IDs**
   - Open `Elevator.java`
   - Update `MOTOR_CAN_ID` to match your motor controller's CAN ID
   - Use Phoenix Tuner or REV Hardware Client to verify/set CAN IDs

3. **Calibrate Physical Constants**
   - Measure your elevator's pulley diameter → Update `PULLEY_DIAMETER_INCHES`
   - Calculate your gear ratio → Update `GEAR_RATIO`
   - Measure max safe extension → Update `MAX_HEIGHT_INCHES`

4. **Set Preset Positions**
   - Measure game element heights on the field
   - Update the `ElevatorPosition` enum values in `Elevator.java`

## ⚙️ Configuration Guide

### Step 1: Hardware Setup

1. **Mount the elevator motor** securely to your robot frame
2. **Connect the motor controller** to the CAN bus
3. **Wire power** from PDP/PDH to motor controller
4. **Verify CAN connection** using Phoenix Tuner or REV Hardware Client

### Step 2: Initial Calibration

```java
// In Elevator.java, update these constants:

// Motor CAN ID (use Phoenix Tuner to verify)
private static final int MOTOR_CAN_ID = 10;

// Mechanical measurements
private static final double PULLEY_DIAMETER_INCHES = 1.5;  // Measure your pulley
private static final double GEAR_RATIO = 12.0;  // Calculate from your gearbox

// Safety limits (CRITICAL - measure carefully!)
private static final double MIN_HEIGHT_INCHES = 0.0;
private static final double MAX_HEIGHT_INCHES = 48.0;  // Your max safe extension
```

### Step 3: PID Tuning

**Initial Tuning Process:**

1. Start with all gains at 0
2. Increase `kP` gradually (start at 0.01, increase to 0.1, 0.5, etc.)
3. Stop when elevator oscillates, then reduce by 50%
4. Add `kD` if oscillations persist (usually 0-0.1)
5. Set `kG` (gravity feedforward) to hold position without drooping (typically 0.03-0.08)

```java
// PID Constants in Elevator.java
private static final double kP = 0.1;   // Proportional gain
private static final double kI = 0.0;   // Usually not needed
private static final double kD = 0.0;   // Damping (if needed)
private static final double kG = 0.05;  // Gravity compensation
```

**Tuning Tips:**
- Use SmartDashboard to monitor position and current
- Test with light load first, then add game element weight
- Ensure elevator doesn't bounce or oscillate at target
- Verify current draw is reasonable (< 40A continuous)

### Step 4: Set Game-Specific Positions

```java
// In Elevator.java, update ElevatorPosition enum:
public enum ElevatorPosition {
    STOWED(0.0),      // Starting configuration
    INTAKE(2.0),      // Ground pickup height
    LOW(12.0),        // Low scoring - measure on field
    MID(24.0),        // Mid scoring - measure on field
    HIGH(42.0);       // High scoring - measure on field
}
```

### Step 5: Controller Configuration

Default button mapping (customizable in `RobotContainer.java`):

**Operator Controller:**
- **A Button** → Move to LOW position
- **B Button** → Move to MID position
- **Y Button** → Move to HIGH position
- **X Button** → Move to STOWED position
- **Right Bumper** → Move to INTAKE position
- **Left Bumper** → Emergency stop
- **Left Stick Y-Axis** → Manual control (default)

**Driver Controller:**
- **D-Pad Up** → Override move up (emergency)
- **D-Pad Down** → Override move down (emergency)

## 🎮 Usage

### Basic Operation

**Preset Positions:**
```java
// In your commands or autonomous:
elevator.setTargetPosition(Elevator.ElevatorPosition.HIGH);

// Wait for elevator to reach position:
Commands.waitUntil(elevator::atTarget);
```

**Manual Control:**
```java
// Direct speed control (-1.0 to 1.0):
elevator.setSpeed(0.5);  // Move up at 50% speed
elevator.setSpeed(-0.3); // Move down at 30% speed
elevator.stop();         // Stop immediately
```

**Custom Positions:**
```java
// Set any position in inches:
elevator.setTargetPosition(15.5);  // Move to 15.5 inches
```

### Autonomous Example

```java
public Command getAutonomousCommand() {
    return Commands.sequence(
        // Move to high position
        Commands.runOnce(() -> elevator.setTargetPosition(ElevatorPosition.HIGH)),
        Commands.waitUntil(elevator::atTarget),
        
        // Wait 2 seconds
        Commands.waitSeconds(2.0),
        
        // Return to stowed
        Commands.runOnce(() -> elevator.setTargetPosition(ElevatorPosition.STOWED)),
        Commands.waitUntil(elevator::atTarget)
    );
}
```

## 🛡️ Safety Features

1. **Soft Limits**: Prevents elevator from extending beyond safe range
2. **Current Limiting**: Protects motor from overcurrent (40A default)
3. **Position Validation**: Clamps all target positions to safe range
4. **Manual Override Protection**: Prevents manual control past limits
5. **Emergency Stop**: Immediate stop capability via button

## 📊 Telemetry & Debugging

SmartDashboard/Shuffleboard provides real-time data:

- **Elevator/Position (in)** - Current height in inches
- **Elevator/Target (in)** - Target position
- **Elevator/Velocity (in per sec)** - Current velocity
- **Elevator/Current (A)** - Motor current draw
- **Elevator/At Target** - Boolean: at target position
- **Elevator/Manual Mode** - Boolean: in manual control
- **Elevator/Motor Output** - Motor output percentage

### Debugging Checklist

✅ **Motor Not Moving:**
- Check CAN ID matches configuration
- Verify motor controller is powered
- Check current draw (should be > 0A when commanding motion)
- Verify encoder is connected and reporting values

✅ **Elevator Oscillates:**
- Reduce `kP` gain by 50%
- Add small `kD` gain (0.01-0.1)
- Check mechanical friction/binding

✅ **Elevator Drifts Down:**
- Increase `kG` (gravity feedforward)
- Check gear ratio and conversion factors
- Verify brake mode is enabled (default in code)

✅ **Position Inaccurate:**
- Verify `PULLEY_DIAMETER_INCHES` is correct
- Verify `GEAR_RATIO` calculation
- Reset encoder at known position
- Check for mechanical slippage

## 🔧 Advanced Features

### Adding Motion Profiling

For smoother, more controlled motion:

```java
// In Elevator.java, replace PID with ProfiledPID:
private final ProfiledPIDController m_controller = new ProfiledPIDController(
    kP, kI, kD,
    new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
);
```

### Adding Limit Switches

```java
// Add to Elevator.java:
private final DigitalInput m_bottomLimit = new DigitalInput(0);
private final DigitalInput m_topLimit = new DigitalInput(1);

// In periodic():
if (m_bottomLimit.get()) {
    resetEncoder();  // Auto-home when bottom limit triggered
}
```

### Current-Based Stall Detection

```java
// Detect if elevator is stuck:
if (m_motor.getOutputCurrent() > 35.0 && Math.abs(getVelocity()) < 0.5) {
    // Elevator is drawing high current but not moving - possible stall
    stop();
}
```

## 📚 Additional Resources

### Official Documentation
- [WPILib Docs](https://docs.wpilib.org/) - Main FRC programming guide
- [Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [PID Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)

### Hardware Documentation
- [REV Spark MAX](https://docs.revrobotics.com/sparkmax/)
- [REV NEO Motor](https://docs.revrobotics.com/brushless)
- [CTRE Phoenix](https://v5.docs.ctr-electronics.com/) (for Falcon motors)

### Community Resources
- [Chief Delphi Forums](https://www.chiefdelphi.com/) - FRC community discussion
- [Team 254 GitHub](https://github.com/Team254) - Example code from top team
- [FRC Discord](https://discord.gg/frc) - Real-time help and discussion

### Video Tutorials
- [FRC Programming Tutorial Series](https://www.youtube.com/playlist?list=PLmP5iMCMnfLQjCDGqQjdSu1w4WxqABXsU)
- [PID Tuning Tutorial](https://www.youtube.com/watch?v=Z24fSBVJeGs)

## 🤝 Contributing

Contributions are welcome! To improve this elevator system:

1. Test thoroughly on your robot
2. Document any changes or improvements
3. Share back with the FRC community
4. Submit issues or pull requests

## 📝 License

This code is provided as an educational resource for FRC teams. Use and modify freely for your team's robot.

## 🏆 Credits

Created for the REBUILT challenge by DRIFT-4091.

Built with WPILib Command-Based framework using industry best practices from successful FRC teams.

## ❓ Troubleshooting & Support

**Common Issues:**

1. **"Motor controller not responding"**
   - Check CAN bus connections
   - Verify CAN ID in code matches physical device
   - Ensure motor controller firmware is updated

2. **"Encoder values not changing"**
   - Verify encoder is properly connected
   - Check if motor is actually spinning
   - Confirm conversion factors are set

3. **"Elevator moves in wrong direction"**
   - Invert motor in code: `m_motor.setInverted(true);`
   - Or swap controller button directions

4. **"Current spikes/breaker trips"**
   - Check for mechanical binding
   - Reduce current limit if needed
   - Verify gear ratio isn't too aggressive

**Getting Help:**
- Check the `notes.md` file for detailed implementation notes
- Post on Chief Delphi forums with your specific issue
- Contact team mentors or FRC community on Discord
- Review WPILib documentation and examples

---

**Good luck with your REBUILT challenge! 🤖⚡**
