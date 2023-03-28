package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * A Command that outtakes.
 */
public class Outtake extends CommandBase {
    private ManipulatorSubsystem m_manipulatorSubsystem;

    /**
     * A command that outtakes.
     *
     * @param manipulatorSubsystem the manipulator subsystem
     */
    public Outtake(ManipulatorSubsystem manipulatorSubsystem) {
        m_manipulatorSubsystem = manipulatorSubsystem;

        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void initialize() {
        m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.OUTTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.HOLD_SPEED);
    }
}
