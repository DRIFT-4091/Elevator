package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorCommands;

/**
 * RobotContainer class for the REBUILT challenge
 * 
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Elevator m_elevator = new Elevator();

    // Controllers
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureBindings();
        
        // Set default commands
        configureDefaultCommands();
    }

    /**
     * Configure default commands for subsystems
     */
    private void configureDefaultCommands() {
        // Default command: Manual elevator control with operator's left stick
        m_elevator.setDefaultCommand(
            Commands.run(
                () -> m_elevator.setSpeed(
                    -m_operatorController.getLeftY() * 0.5  // Reduced speed for safety
                ),
                m_elevator
            ).withName("ManualElevatorControl")
        );
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController} or other controller classes.
     */
    private void configureBindings() {
        // Operator Controls for Elevator
        
        // A Button: Move to LOW position
        m_operatorController.a()
            .onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.ElevatorPosition.LOW), m_elevator)
                .andThen(Commands.waitUntil(m_elevator::atTarget))
                .withName("MoveToLow"));

        // B Button: Move to MID position
        m_operatorController.b()
            .onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.ElevatorPosition.MID), m_elevator)
                .andThen(Commands.waitUntil(m_elevator::atTarget))
                .withName("MoveToMid"));

        // Y Button: Move to HIGH position
        m_operatorController.y()
            .onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.ElevatorPosition.HIGH), m_elevator)
                .andThen(Commands.waitUntil(m_elevator::atTarget))
                .withName("MoveToHigh"));

        // X Button: Move to STOWED position
        m_operatorController.x()
            .onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.ElevatorPosition.STOWED), m_elevator)
                .andThen(Commands.waitUntil(m_elevator::atTarget))
                .withName("MoveToStowed"));

        // Right Bumper: Move to INTAKE position
        m_operatorController.rightBumper()
            .onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.ElevatorPosition.INTAKE), m_elevator)
                .andThen(Commands.waitUntil(m_elevator::atTarget))
                .withName("MoveToIntake"));

        // Left Bumper: Emergency stop
        m_operatorController.leftBumper()
            .onTrue(Commands.runOnce(() -> m_elevator.stop(), m_elevator)
                .withName("EmergencyStop"));

        // Driver can also control elevator for emergency override
        // D-Pad Up: Override move up slowly
        m_driverController.povUp()
            .whileTrue(Commands.run(() -> m_elevator.setSpeed(0.3), m_elevator)
                .withName("DriverOverrideUp"));

        // D-Pad Down: Override move down slowly
        m_driverController.povDown()
            .whileTrue(Commands.run(() -> m_elevator.setSpeed(-0.3), m_elevator)
                .withName("DriverOverrideDown"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Example autonomous command: Move elevator to high position then back to stowed
        return Commands.sequence(
            Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.ElevatorPosition.HIGH), m_elevator),
            Commands.waitUntil(m_elevator::atTarget),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.ElevatorPosition.STOWED), m_elevator),
            Commands.waitUntil(m_elevator::atTarget)
        ).withName("AutoElevatorRoutine");
    }

    /**
     * Get the elevator subsystem for testing or external access
     * 
     * @return the elevator subsystem
     */
    public Elevator getElevator() {
        return m_elevator;
    }
}
