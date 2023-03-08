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

    public AutoLevel(DrivetrainSubsystem drive) {
        m_xSpeed = SmartDashboard.getNumber("Robot AutoLevel X Speed", 0);
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
        m_xSpeed = SmartDashboard.getNumber("Robot AutoLevel X Speed", 0);
        m_deg = getRobotPitch().getDegrees();
        m_lastTime = RobotController.getFPGATime();
        m_lastPitch = getRobotPitch().getDegrees();
    }

    @Override
    public void execute() {
        // double p = getRobotPitch().getDegrees();
        // long t = RobotController.getFPGATime();
        // double newV = (p - m_lastPitch) / (t - m_lastTime) * 1000000;
        // double v = (m_lastVel + newV) / 2;
        // if (Math.abs(v) > SmartDashboard.getNumber("Robot AutoLevel Angular Velocity Cutoff", 10)) {
        //     if (m_sign == null) {
        //         if (Math.signum(v) == 1.0) {
        //             m_sign = SignChange.POSITIVE;
        //         }
        //         else {
        //             m_sign = SignChange.NEGATIVE;
        //         }
        //     }
        //     else {
        //         if (Math.signum(v) == -1.0 && m_sign == SignChange.POSITIVE && Math.abs(p) < 20) {
        //             end(false);
        //             return;
        //         } else if (Math.signum(v) == 1.0 && m_sign == SignChange.NEGATIVE && Math.abs(p) < 20) {
        //             end(false);
        //             return;
        //         }
        //     }
        // }
        // m_drive.drive(m_xSpeed, 0, 0, true);
        // m_lastTime = t;
        // m_lastPitch = p;
        // m_lastVel = v;
        if (Math.abs(getRobotPitch().getDegrees()) > SmartDashboard.getNumber("Robot AutoLevel Docked Trigger", 15) && !m_onStation) {
            m_onStation = true;
        }
        m_drive.drive(m_xSpeed, 0, 0, true);
        if (Math.abs(getRobotPitch().getDegrees()) < SmartDashboard.getNumber("Robot AutoLevel Engaged Trigger", 5) && m_onStation) {
            end(false);
            return;
        }

        SmartDashboard.putNumber("Calculated Robot Angle", m_deg);
        // SmartDashboard.putNumber("Calculated Robot Anglular Velocity", v);
    }

    public Rotation2d getRobotPitch() {
        // This is probably not the correct way to do this, but it's a close enough approximation
        double roll = m_drive.getGyroRoll().getRadians();
        double pitch = m_drive.getGyroPitch().getRadians();
        double yaw = m_drive.getGyroYaw().getRadians();
        double theNumber = Math.cos(yaw) * pitch + Math.sin(yaw) * roll;
        return Rotation2d.fromRadians(theNumber);
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
