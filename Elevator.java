package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final TalonFX motor;

    // How far you want the arms to move (in motor rotations)
    // Start small and increase later
    private static final double EXTEND_ROTATIONS = 20.0;

    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private boolean extended = false;

    public Elevator() {
        motor = new TalonFX(10);  //Tune this id 

        // Start retracted
        motor.setPosition(0);
    }

    public void extend() {
        motor.setControl(positionRequest.withPosition(EXTEND_ROTATIONS));
        extended = true;
    }

    public void retract() {
        motor.setControl(positionRequest.withPosition(0.0));
        extended = false;
    }

    public void toggle() {
        if (extended) {
            retract();
        } else {
            extend();
        }
    }

    public boolean isExtended() {
        return extended;
    }
}