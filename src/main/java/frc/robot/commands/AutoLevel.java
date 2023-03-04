package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoLevelConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLevel extends CommandBase {
    private final DrivetrainSubsystem m_drive;
    private final PIDController m_pid;
    private final Timer m_timer;
    private double m_deg;
    public AutoLevel(DrivetrainSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
        m_timer = new Timer();
        m_pid = new PIDController(AutoLevelConstants.kP, AutoLevelConstants.kI, AutoLevelConstants.kD);
        m_pid.setTolerance(AutoLevelConstants.tolerance);
        SmartDashboard.putNumber("Robot AutoLevel P", AutoLevelConstants.kP);
        SmartDashboard.putNumber("Robot AutoLevel D", AutoLevelConstants.kD);
        // SmartDashboard.putNumber("Robot AutoLevel X Speed", 1);
    }

    @Override
    public void initialize() {
        m_pid.setSetpoint(0);
        m_deg = getRobotPitch().getDegrees();
    }

    @Override
    public void execute() {
        double newP = SmartDashboard.getNumber("Robot AutoLevel P", AutoLevelConstants.kP);
        double newD = SmartDashboard.getNumber("Robot AutoLevel D", AutoLevelConstants.kD);
        // double newX = SmartDashboard.getNumber("Robot AutoLevel X Speed", 1);
        m_deg = getRobotPitch().getDegrees();
        m_pid.setP(newP);
        m_pid.setD(newD);
        if (m_pid.atSetpoint()) {
            m_pid.setSetpoint(m_deg);
            m_drive.drive(0, 0, 0, true);
        } else {
            double value = m_pid.calculate(m_deg, 0);
            m_drive.drive(value, 0, 0, true);
        }
        // if (Math.abs(m_deg) > AutoLevelConstants.tolerance) {
        //     m_drive.drive(-Math.signum(m_deg) * newX, 0, 0, true);
        // }
        // else {
        //     m_drive.drive(0, 0, 0, true);
        // }
        SmartDashboard.putNumber("Calculated Robot Angle", m_deg);
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
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_deg) < AutoLevelConstants.tolerance) {
            m_timer.start();
            return m_timer.get() > AutoLevelConstants.waitTime;
        }
        else {
            m_timer.stop();
            m_timer.reset();
            return false;
        }
    }
}
