package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A command that uses the vision markers on the nodes to align the robot with said nodes.
 */
public class LLAlignment extends CommandBase {

    private final DrivetrainSubsystem m_drive;
    private final int m_pipeline;
    private final double m_strafeDeadband = 0.05;
    private final double m_strafekP = 0.05;
    private final double m_turnDeadband = Math.toRadians(3);
    private final double m_driverDeadband = 0.1;
    private final PIDController m_turnPID = new PIDController(4.5, 0, 0);
    private final XboxController m_driver;
    private double m_tx;
    private boolean m_tv;

    /**
     * Constructs an instance of this command.
     */
    public LLAlignment(int pipelineId, XboxController driver, DrivetrainSubsystem drive) {
        m_drive = drive;
        m_pipeline = pipelineId;
        m_driver = driver;
        m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
        m_turnPID.setTolerance(m_turnDeadband);
        m_turnPID.setSetpoint(Math.PI);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_turnPID.reset();
        m_turnPID.setSetpoint(Math.PI);
        LimelightHelpers.setPipelineIndex("limelight", m_pipeline);
        LimelightHelpers.setCameraMode_Processor("limelight");
        m_tx = LimelightHelpers.getTX("limelight");
        m_tv = LimelightHelpers.getTV("limelight");
    }

    @Override
    public void execute() {
        m_tx = LimelightHelpers.getTX("limelight");
        m_tv = LimelightHelpers.getTV("limelight");
        Pose2d robotPose = m_drive.getPose();
        double turnPIDOut = m_turnPID.calculate(robotPose.getRotation().getRadians());
        if (m_turnPID.atSetpoint()) {
            if (m_tv) {
                m_driver.setRumble(RumbleType.kBothRumble, 0);
                if (Math.abs(m_tx) > m_strafeDeadband) {
                    m_drive.drive(applyDeadband(m_driver.getLeftY()), -(m_tx * m_strafekP), 0, false, false);
                } else {
                    m_drive.drive(applyDeadband(m_driver.getLeftY()), 0, 0, false, false);
                }
            }
            else {
                m_driver.setRumble(RumbleType.kBothRumble, 0.5);
            }
        }
        else {
            m_drive.drive(0, 0, turnPIDOut, false, false);
        }
    }

    private double applyDeadband(double in) {
        return Math.abs(in) >= m_driverDeadband ? in : 0.0;
    }
    
    @Override
    public boolean isFinished() {
        // Pose2d robotPose = m_drive.getPose();
        // double roundedRad = robotPose.getRotation().getRadians() % (2 * Math.PI);
        // return (Math.abs(roundedRad - Math.PI) < m_turnDeadband) && (Math.abs(m_tx) < m_strafeDeadband) && m_tv;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex("limelight", 3);
        LimelightHelpers.setCameraMode_Driver("limelight");
        m_drive.drive(0, 0, 0, false);
        m_driver.setRumble(RumbleType.kBothRumble, 0);
    }
}