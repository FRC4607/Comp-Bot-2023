package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLevel extends CommandBase {
    private final DrivetrainSubsystem m_drive;
    private double m_deg;

    private double m_xSpeed;

    private Pose2d m_startPose;
    
    private long m_lastTime;
    private double m_lastPitch;
    private double m_lastVel = 0;

    private boolean m_end = false;

    private enum SignChange {
        POSITIVE,
        NEGATIVE
    };

    private SignChange m_sign = null;

    private boolean m_onStation = false;

    public AutoLevel(double xSpeed, DrivetrainSubsystem drive) {
        m_xSpeed = xSpeed;
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_onStation = false;
        m_startPose = m_drive.getPose();
        m_lastVel = 0;
        m_end = false;
        m_sign = null;
        m_deg = m_drive.getRobotPitch().getDegrees();
        m_lastTime = RobotController.getFPGATime();
        m_lastPitch = m_drive.getRobotPitch().getDegrees();
    }

    @Override
    public void execute() {
        if (Math.abs(m_drive.getRobotPitch().getDegrees()) > 20 && !m_onStation) {
            m_onStation = true;
        }
        if (Math.abs(m_drive.getRobotPitch().getDegrees()) < 11 && m_onStation) {
            end(false);
        } else if (m_onStation) {
            m_drive.drive(m_xSpeed, 0, 0, true);
        } else {
            m_drive.drive(m_xSpeed * 2, 0, 0, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
        m_end = true;
    }

    @Override
    public boolean isFinished() {

        if (m_end || m_drive.getPose().minus(m_startPose).getTranslation().getNorm() > 3) {
            m_end = false;
            return true;
        }
        else {
            return false;
        }
    }
}