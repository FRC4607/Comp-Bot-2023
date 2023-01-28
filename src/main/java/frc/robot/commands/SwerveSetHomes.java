package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A Command that Sets the Current Positions of the swerve modules as home.
 */
public class SwerveSetHomes extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    /**
     * A Command that Sets the Current Positions of the swerve modules as home. The
     * wheels should be lined up so that they are parallel to forward with the bevel
     * gear on the inside.
     *
     * @param drivetrainSubsystem The swerve drivetrain subsystem
     */
    public SwerveSetHomes(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setModuleHomes();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
