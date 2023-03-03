package frc.robot.commands;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoLevelConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLevel extends CommandBase {
    private final DrivetrainSubsystem m_drive;
    private final PIDController m_pid;
    private final Timer m_timer;
    private final MatBuilder<N3, N3> m_matBuilder = new MatBuilder<>(Nat.N3(), Nat.N3());

    public AutoLevel(DrivetrainSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
        m_timer = new Timer();
        m_pid = new PIDController(AutoLevelConstants.kP, AutoLevelConstants.kI, AutoLevelConstants.kD);
        m_pid.setTolerance(AutoLevelConstants.tolerance);
        SmartDashboard.putNumber("Robot AutoLevel P", AutoLevelConstants.kP);
        SmartDashboard.putNumber("Robot AutoLevel D", AutoLevelConstants.kD);
    }

    @Override
    public void initialize() {
        m_pid.setSetpoint(0);
    }

    @Override
    public void execute() {
        double newP = SmartDashboard.getNumber("Robot AutoLevel P", AutoLevelConstants.kP);
        double newD = SmartDashboard.getNumber("Robot AutoLevel D", AutoLevelConstants.kD);
        m_pid.setP(newP);
        m_pid.setD(newD);
        // if (m_pid.atSetpoint()) {
        //     m_pid.setSetpoint(getXRelativeRobotAngle().getDegrees());
        //     m_drive.drive(0, 0, 0, true);
        // } else {
        //     double value = m_pid.calculate(getXRelativeRobotAngle().getDegrees(), 0);
        //     m_drive.drive(value, 0, 0, true);
        // }
        SmartDashboard.putNumber("Calculated Robot Angle", getRobotTiltAngle().getDegrees());
    }

    public Rotation2d getRobotTiltAngle() {
        // Vector/Matrix Magic
        // See https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
        // and https://www.cuemath.com/geometry/angle-between-vectors/
        // We know the floor's normal already
        Vector<N3> floorNormal = VecBuilder.fill(0, 0, 1);
        // Construct the rotation matrices
        Matrix<N3, N3> xRot = m_matBuilder.fill(
            1.0, 0.0, 0.0,
            0.0, Math.cos(m_drive.getPigeonRollRotation().getRadians()), -Math.sin(m_drive.getPigeonRollRotation().getRadians()),
            0.0, Math.sin(m_drive.getPigeonRollRotation().getRadians()), Math.cos(m_drive.getPigeonRollRotation().getRadians())
        );
        Matrix<N3, N3> yRot = m_matBuilder.fill(
            Math.cos(m_drive.getPigeonPitchRotation().getRadians()), 0.0, Math.sin(m_drive.getPigeonPitchRotation().getRadians()),
            0.0, 1.0, 0.0,
            -Math.sin(m_drive.getPigeonPitchRotation().getRadians()), 0.0, Math.cos(m_drive.getPigeonPitchRotation().getRadians())
        );
        // Rotate the floor normal by the rotation matrices to get the normal of the robot's plane
        Vector<N3> robotNormal = new Vector<N3>(yRot.times(xRot.times(floorNormal)));
        // Find the angle between the two vectors
        double theta = Math.acos(robotNormal.dot(floorNormal)) / (robotNormal.norm() * floorNormal.norm());
        // Package into a Rotation2d
        return Rotation2d.fromRadians(theta);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        // if (m_pid.atSetpoint()) {
        //     m_timer.start();
        //     return m_timer.get() > AutoLevelConstants.waitTime;
        // }
        // else {
        //     m_timer.stop();
        //     m_timer.reset();
        //     return false;
        // }
        return false;
    }
}
