package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLevel extends CommandBase {
    private final DrivetrainSubsystem m_drive;

    private final double m_xSpeed;

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
        if (Math.abs(m_drive.getRobotPitch().getDegrees() - 1.6) < 9) {
            System.out.println("end criteria met, setting m_end");
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
        m_sign = (m_drive.getRobotPitch().getDegrees() - 1.6) > 0 ? SignChange.POSITIVE : SignChange.NEGATIVE;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
        m_end = true;
    }

    @Override
    public boolean isFinished() {
        if (m_end
            || (m_drive.getPose().minus(m_startPose).getTranslation().getNorm() > 3)
            || (DriverStation.getMatchTime() < 0.075)) {
            if (m_end) {
                System.out.println("Ended because m_end was set to true");
            } else if (m_drive.getPose().minus(m_startPose).getTranslation().getNorm() > 3) {
                System.out.println("Ended due to 3 meter failsafe");
            } else {
                System.out.println("Ended due to end of period stop");;
            }
            m_end = false;
            return true;
        } else {
            return false;
        }
    }
}