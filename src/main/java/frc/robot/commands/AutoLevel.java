package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLevel extends CommandBase {
    private final DrivetrainSubsystem m_drive;
    private double m_deg;

    private double m_xSpeed;

    private long m_lastTime;
    private double m_lastPitch;

    private boolean m_end = false;

    private enum SignChange {
        POSITIVE,
        NEGATIVE
    };

    private SignChange m_sign = null;

    public AutoLevel(DrivetrainSubsystem drive) {
        m_xSpeed = SmartDashboard.getNumber("Robot AutoLevel X Speed", 0);
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_xSpeed = SmartDashboard.getNumber("Robot AutoLevel X Speed", 0);
        m_deg = getRobotPitch().getDegrees();
        m_lastTime = RobotController.getFPGATime();
        m_lastPitch = getRobotPitch().getDegrees();
    }

    @Override
    public void execute() {
        double p = getRobotPitch().getDegrees();
        long t = RobotController.getFPGATime();
        double v = (p - m_lastPitch) / (t - m_lastTime);
        if (Math.abs(v) > SmartDashboard.getNumber("Robot AutoLevel Angular Velocity Cutoff", 10)) {
            if (m_sign == null) {
                if (Math.signum(v) == 1.0) {
                    m_sign = SignChange.POSITIVE;
                }
                else {
                    m_sign = SignChange.NEGATIVE;
                }
            }
            else {
                if (Math.signum(v) == -1.0 && m_sign == SignChange.POSITIVE) {
                    end(false);
                    return;
                } else if (Math.signum(v) == 1.0 && m_sign == SignChange.NEGATIVE) {
                    end(false);
                    return;
                }
            }
        }
        m_drive.drive(m_xSpeed, 0, 0, true);
        m_lastTime = t;
        m_lastPitch = p;
        SmartDashboard.putNumber("Calculated Robot Angle", m_deg);
        SmartDashboard.putNumber("Calculated Robot Anglular Velocity", v);
    }

    public Rotation2d getRobotPitch() {
        // This is probably not the correct way to do this, but it's a close enough approximation
        double roll = m_drive.getPigeonRollRotation().getRadians();
        double pitch = m_drive.getPigeonPitchRotation().getRadians();
        double yaw = m_drive.getPigeonYawRotation().getRadians();
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
        return m_end;
    }
}
