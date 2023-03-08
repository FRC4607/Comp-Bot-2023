package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Implements a command to collect data to log the robot's pitch for analysis.
 */
public class RobotPitchLog extends CommandBase {

    private enum State {
        BEFORE_CHARGE_STATION,
        DOCKED_PRE_ENGAGED,
        ENGAGED,
        DOCKED_POST_ENGAGE,
        AFTER_CHARGE_STATION
    }

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private final double m_driveVoltage = 6.0;
    private boolean m_isFinished;
    private State m_state;
    private int m_clearChargeStationCounter;
    // 2 seconds * 20 loops/second = 40 loops
    private final int m_clearChargeStationThreshold = 40;

    /**
     * Constructor for the calibrate tunning feed-forward command.
     *
     * @param drivetrainSubsystem uses the drivetrain subsystem
     */
    public RobotPitchLog(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }

    /**
     * Update the drivetrain state to indicate the subsystem is calibrating the modules.
     * 
     * <p>This command should be scheduled as non-interruptible.
     */
    @Override
    public void initialize() {
        m_isFinished = false;
        m_state = State.BEFORE_CHARGE_STATION;
        DriverStation.reportWarning("Starting the turning motors feed-forward calibration", false);

        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModuleStates[i] = new SwerveModuleState();
        }

        m_drivetrainSubsystem.setModuleStates(swerveModuleStates, false);
    }

    /**
     * Run the calibration routine on each of the drivetrain swerve modules.
     */
    @Override
    public void execute() {
        switch (m_state) {
            case BEFORE_CHARGE_STATION:
                break;
                
            case DOCKED_PRE_ENGAGED:
                break;

            case ENGAGED:
                break;
            
            case DOCKED_POST_ENGAGE:
                break;
            
            case AFTER_CHARGE_STATION:
                m_clearChargeStationCounter++;
                if (m_clearChargeStationCounter >= m_clearChargeStationThreshold) {
                    m_isFinished = true;
                    break;
                }
                break;
        }

        m_drivetrainSubsystem.setModuleDriveVoltage(m_driveVoltage);

    }

    /**
     * The command is complete when all the stages are complete.
     */
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    /**
     * Update the drivetrain state when all of the modules have completed the Calibration.
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.setModuleDriveVoltage(0);
        DriverStation.reportWarning("Stopping the turning motors feed-forward calibration", false);
    }

}
