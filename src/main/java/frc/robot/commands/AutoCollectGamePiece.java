package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * A command to collect game piece in auto. Not fully functional
 */
public class AutoCollectGamePiece extends CommandBase {

    private ManipulatorSubsystem m_manipulatorSubsystem;
    private int m_counter;
    private boolean m_timeout;

    /**
     * A command to collect game piece in auto.
     *
     * @param manipulatorSubsystem The Manipulator Subsystem
     */
    public AutoCollectGamePiece(ManipulatorSubsystem manipulatorSubsystem, boolean timeout) {
        m_manipulatorSubsystem = manipulatorSubsystem;
        m_timeout = timeout;
        addRequirements(m_manipulatorSubsystem);
    }

    @Override
    public void initialize() {
        m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.INTAKE_SPEED);
        m_counter = 0;
        DataLogManager.log("Piece Collection Stared, m_timeout = " + m_timeout);
    }

    @Override
    public void execute() {
        m_counter++;

        if (m_counter > 50 && m_timeout) {
            DataLogManager.log("Counter exceeded");
            m_manipulatorSubsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.HOLD_SPEED);
        DataLogManager.log("Piece Collection Ended, m_timeout = " + m_timeout);
    }

    @Override
    public boolean isFinished() {
        return m_manipulatorSubsystem.getCurrent() > ManipulatorCalibrations.PIECE_DETECTION_CURRENT;
                // && Math.abs(m_manipulatorSubsystem.getSpeed()) < ManipulatorCalibrations.PIECE_DETECTION_RPM;
    }

}
