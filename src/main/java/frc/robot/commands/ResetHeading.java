package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A command to Set the current heading as forward.
 */
public class ResetHeading extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    /**
     * A command to Set the current heading as forward.
     *
     * @param drivetrainSubsystem the swerve drivetrain
     */
    public ResetHeading(DrivetrainSubsystem drivetrainSubsystem) {

        m_drivetrainSubsystem = drivetrainSubsystem;
    
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.resetHeading();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
