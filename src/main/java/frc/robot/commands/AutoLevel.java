package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLevel extends CommandBase {
    private final DrivetrainSubsystem m_drive;

    private double m_xSpeed;

    private Pose2d m_startPose;

    private boolean m_end = false;

    private enum SignChange {
        POSITIVE,
        NEGATIVE
    };

    private SignChange m_sign = null;

    private long m_startTime;

    public AutoLevel(double xSpeed, DrivetrainSubsystem drive) {
        m_xSpeed = xSpeed;
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_startPose = m_drive.getPose();
        m_end = false;
        m_sign = m_drive.getRobotPitch().getDegrees() > 0 ? SignChange.POSITIVE : SignChange.NEGATIVE;
        m_startTime = RobotController.getFPGATime();
    }

    @Override
    public void execute() {
        if (Math.abs(m_drive.getRobotPitch().getDegrees() - 1.14) < 13.5) {
            end(false);
            return;
        } else {
            m_end = false;
        }
        switch (m_sign) {
            case POSITIVE: 
                m_drive.drive(m_xSpeed, 0, 0, true);
                break;
            case NEGATIVE: 
                m_drive.drive(-m_xSpeed, 0, 0, true);
                break;
            default:
                break;
        }
        m_sign = m_drive.getRobotPitch().getDegrees() > 0 ? SignChange.POSITIVE : SignChange.NEGATIVE;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
        m_end = true;
    }

    @Override
    public boolean isFinished() {
        if ((m_end && (RobotController.getFPGATime() - m_startTime) > (1.0 * 1e6))
            || (m_drive.getPose().minus(m_startPose).getTranslation().getNorm() > 3)
            || (DriverStation.getMatchTime() < 0.75)) {
            m_end = false;
            return true;
        } else {
            return false;
        }
    }
}