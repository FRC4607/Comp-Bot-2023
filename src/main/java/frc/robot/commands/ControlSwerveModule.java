package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ControlSwerveModule extends CommandBase {

    private int m_module;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public ControlSwerveModule(int module, DrivetrainSubsystem drivetrainSubsystem) {

        m_module = module;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);

    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Drive Target", 0.0);
        SmartDashboard.putNumber("Turn Target (deg)", 0.0);
    }

    @Override
    public void execute() {
        SwerveModuleState[] states = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        states[m_module] = new SwerveModuleState(
            SmartDashboard.getNumber("Drive Target", 0),
            Rotation2d.fromDegrees(SmartDashboard.getNumber("Turn Target (deg)", 0))
        );

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
