package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Elevator subsystem for the REBUILT challenge
 * 
 * Controls an elevator mechanism using a NEO motor with PID position control.
 * Supports preset positions and manual control modes.
 */
public class Elevator extends SubsystemBase {
    // Hardware
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_pidController;

    // Feedforward for gravity compensation
    private final SimpleMotorFeedforward m_feedforward;

    // Constants - ADJUST THESE FOR YOUR ROBOT
    private static final int MOTOR_CAN_ID = 10;  // Change to match your robot's CAN ID
    
    // Conversion factors (rotations to inches) - CALIBRATE FOR YOUR ROBOT
    private static final double PULLEY_DIAMETER_INCHES = 1.5;  // Diameter of elevator pulley
    private static final double GEAR_RATIO = 12.0;  // Motor rotations per pulley rotation
    private static final double POSITION_CONVERSION_FACTOR = 
        (PULLEY_DIAMETER_INCHES * Math.PI) / GEAR_RATIO;  // Inches per motor rotation
    
    // Soft limits (in inches) - CALIBRATE FOR YOUR ROBOT
    private static final double MIN_HEIGHT_INCHES = 0.0;
    private static final double MAX_HEIGHT_INCHES = 48.0;  // Maximum safe extension
    
    // PID Constants - TUNE THESE VALUES
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;  // Feedforward for velocity
    private static final double kG = 0.05;  // Feedforward for gravity (static hold)
    
    // Motion constraints
    private static final double MAX_VELOCITY = 60.0;  // inches per second
    private static final double MAX_ACCELERATION = 40.0;  // inches per second^2
    
    // Tolerance for position control
    private static final double POSITION_TOLERANCE_INCHES = 0.5;
    
    // Current limits
    private static final int CURRENT_LIMIT_AMPS = 40;

    /**
     * Preset elevator positions for common game tasks
     */
    public enum ElevatorPosition {
        STOWED(0.0),      // Fully retracted
        INTAKE(2.0),      // Low position for intake
        LOW(12.0),        // Low scoring position
        MID(24.0),        // Mid scoring position
        HIGH(42.0);       // High scoring position

        public final double heightInches;

        ElevatorPosition(double heightInches) {
            this.heightInches = heightInches;
        }
    }

    // State
    private double m_targetPosition = 0.0;
    private boolean m_isManualControl = false;

    /**
     * Creates a new Elevator subsystem
     */
    public Elevator() {
        // Initialize motor
        m_motor = new CANSparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        
        // Configure current limiting
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
        
        // Get encoder and PID controller
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        
        // Configure encoder conversion factor
        m_encoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        m_encoder.setVelocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);  // RPM to inches/sec
        
        // Reset encoder to 0 (assumes elevator starts at bottom)
        m_encoder.setPosition(0.0);
        
        // Configure PID
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(-1.0, 1.0);
        
        // Set up feedforward for gravity compensation
        m_feedforward = new SimpleMotorFeedforward(kG, 0.0, 0.0);
        
        // Set initial target to current position
        m_targetPosition = m_encoder.getPosition();
        
        // Burn flash to save configuration
        m_motor.burnFlash();
    }

    /**
     * Move elevator to a preset position
     * 
     * @param position The preset position to move to
     */
    public void setTargetPosition(ElevatorPosition position) {
        setTargetPosition(position.heightInches);
    }

    /**
     * Move elevator to a specific height in inches
     * 
     * @param heightInches Target height in inches (will be clamped to soft limits)
     */
    public void setTargetPosition(double heightInches) {
        m_isManualControl = false;
        m_targetPosition = clampToLimits(heightInches);
    }

    /**
     * Set elevator speed for manual control (-1.0 to 1.0)
     * 
     * @param speed Motor speed from -1.0 (down) to 1.0 (up)
     */
    public void setSpeed(double speed) {
        m_isManualControl = true;
        
        // Check soft limits during manual control
        double currentPosition = m_encoder.getPosition();
        
        if (currentPosition <= MIN_HEIGHT_INCHES && speed < 0) {
            // At bottom limit, don't allow downward movement
            speed = 0.0;
        } else if (currentPosition >= MAX_HEIGHT_INCHES && speed > 0) {
            // At top limit, don't allow upward movement
            speed = 0.0;
        }
        
        m_motor.set(speed);
    }

    /**
     * Stop the elevator immediately
     */
    public void stop() {
        m_isManualControl = false;
        m_targetPosition = m_encoder.getPosition();
        m_motor.set(0.0);
    }

    /**
     * Check if elevator is at target position
     * 
     * @return true if within tolerance of target
     */
    public boolean atTarget() {
        return Math.abs(m_encoder.getPosition() - m_targetPosition) < POSITION_TOLERANCE_INCHES;
    }

    /**
     * Get current elevator position in inches
     * 
     * @return current height in inches
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Get current elevator velocity in inches per second
     * 
     * @return velocity in inches/sec
     */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    /**
     * Get motor current draw
     * 
     * @return current in amps
     */
    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    /**
     * Reset encoder to 0 (call when elevator is at known bottom position)
     */
    public void resetEncoder() {
        m_encoder.setPosition(0.0);
        m_targetPosition = 0.0;
    }

    /**
     * Clamp a height value to the soft limits
     * 
     * @param heightInches Desired height
     * @return Clamped height within limits
     */
    private double clampToLimits(double heightInches) {
        return Math.max(MIN_HEIGHT_INCHES, Math.min(MAX_HEIGHT_INCHES, heightInches));
    }

    @Override
    public void periodic() {
        // This method is called once per scheduler run (every 20ms)
        
        if (!m_isManualControl) {
            // Use PID control to reach target position
            // Add feedforward for gravity compensation
            double feedforwardOutput = m_feedforward.calculate(0.0);  // Static feedforward for gravity
            m_pidController.setReference(
                m_targetPosition, 
                CANSparkMax.ControlType.kPosition,
                0,  // PID slot
                feedforwardOutput
            );
        }
        
        // Update telemetry
        updateTelemetry();
    }

    /**
     * Update SmartDashboard with current elevator status
     */
    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator/Position (in)", m_encoder.getPosition());
        SmartDashboard.putNumber("Elevator/Target (in)", m_targetPosition);
        SmartDashboard.putNumber("Elevator/Velocity (in per sec)", m_encoder.getVelocity());
        SmartDashboard.putNumber("Elevator/Current (A)", m_motor.getOutputCurrent());
        SmartDashboard.putBoolean("Elevator/At Target", atTarget());
        SmartDashboard.putBoolean("Elevator/Manual Mode", m_isManualControl);
        SmartDashboard.putNumber("Elevator/Motor Output", m_motor.getAppliedOutput());
    }

    /**
     * Initialize elevator for use (call during robot init)
     */
    public void initialize() {
        // Could add homing routine here if you have limit switches
        resetEncoder();
    }
}
