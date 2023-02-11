package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A command that Allows control of individual modules via SmartDashboard.
 */
public class ControlSwerveModule extends CommandBase {

    private DrivetrainSubsystem m_drivetrainSubsystem;

    /**
     * A command that Allows control of individual modules via SmartDashboard.
     *
     * @param drivetrainSubsystem the Swerve Drivetrain Subsystem
     */
    public ControlSwerveModule(DrivetrainSubsystem drivetrainSubsystem) {

        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);

    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Drive Target", 0.0);
        SmartDashboard.putNumber("Turn Target (deg)", 0.0);
        SmartDashboard.putNumber("Module Number", 0);
    }

    @Override
    public void execute() {
        SwerveModuleState[] states = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        int module = (int) SmartDashboard.getNumber("Module Number", 0);

        states[module % 4] = new SwerveModuleState(
                SmartDashboard.getNumber("Drive Target", 0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber("Turn Target (deg)", 0)));

        m_drivetrainSubsystem.setModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveModuleState[] states = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
        m_drivetrainSubsystem.setModuleStates(states);
    }
}
