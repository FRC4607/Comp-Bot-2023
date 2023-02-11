package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.WristSubsystem;

/**
 * The command that moves the Wrist.
 */
public class MoveWrist extends CommandBase {
    WristSubsystem m_wristSubsystem;
    BooleanSupplier m_dPadUp;
    BooleanSupplier m_dPadDown;

    public MoveWrist(BooleanSupplier dPadUp, BooleanSupplier dPadDown, WristSubsystem wristSubsystem) {
        m_dPadUp = dPadUp;
        m_dPadDown = dPadDown;

        m_wristSubsystem = wristSubsystem;
        addRequirements(m_wristSubsystem);

    }

    @Override
    public void execute() {
        
        if (m_dPadUp.getAsBoolean()) {

            m_wristSubsystem.moveWrist(ManipulatorConstants.WRIST_MOTOR_SPEED);

        } else if (m_dPadDown.getAsBoolean()) {

            m_wristSubsystem.moveWrist(-(ManipulatorConstants.WRIST_MOTOR_SPEED));

        } else {

            m_wristSubsystem.moveWrist(0);

        }
    }
}