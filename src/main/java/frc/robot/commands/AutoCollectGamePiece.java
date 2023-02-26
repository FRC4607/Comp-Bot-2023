package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * A command to collect game piece in auto. Not fully functional
 */
public class AutoCollectGamePiece extends CommandBase {
    private ElevatorSubsystem m_elevatorSubsystem;
    private ArmSubsystem m_armSubsystem;
    private ManipulatorSubsystem m_manipulatorSubsystem;

    private State m_state;
    private int m_counter;

    private enum State {
        extendingElevator,
        extendingArm,
        collectingPiece,
        retractingArm,
        retractingElevator,
        done
    }

    /**
     * A command to collect game piece in auto.
     *
     * @param elevatorSubsystem The Elevator Subsystem
     * @param armSubsystem The Arm Subsystem
     * @param manipulatorSubsystem The Manipulator Subsystem
     */
    public AutoCollectGamePiece(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem,
            ManipulatorSubsystem manipulatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;

        addRequirements(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    }

    @Override
    public void initialize() {
        m_state = State.extendingElevator;
        m_counter = 0;
        m_elevatorSubsystem.setElevatorPosition(ElevatorCalibrations.COLLECTION_POSITION + ElevatorCalibrations.TOLERANCE);
        m_elevatorSubsystem.resetController();
        m_armSubsystem.setArmTargetPosition(ArmCalibrations.POSITION_RETRACTED);
        m_armSubsystem.resetController();
    }

    @Override
    public void execute() {
        switch (m_state) {
            case extendingElevator:
                if (Math.abs(m_elevatorSubsystem.getEncoderPosition()
                        - ElevatorCalibrations.COLLECTION_POSITION) < ElevatorCalibrations.TOLERANCE) {

                    m_armSubsystem.setArmTargetPosition(ArmCalibrations.POSITION_PIECE_COLLECTION);
                    m_state = State.extendingArm;
                }
                break;

            case extendingArm:
                if (Math.abs(m_armSubsystem.getAbsoluteEncoderPosition()
                        - ArmCalibrations.POSITION_PIECE_COLLECTION) < ArmCalibrations.TOLERANCE) {

                    m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.INTAKE_SPEED);
                    m_state = State.collectingPiece;
                }
                break;

            case collectingPiece:
                if (m_counter > 150) {
                    m_armSubsystem.setArmTargetPosition(ArmCalibrations.POSITION_RETRACTED);
                    m_state = State.retractingArm;
                    m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.HOLD_SPEED);
                }
                m_counter++;
                break;

            case retractingArm:
                if (Math.abs(m_armSubsystem.getAbsoluteEncoderPosition()
                        - ArmCalibrations.POSITION_RETRACTED) < ArmCalibrations.TOLERANCE) {
                    m_elevatorSubsystem.setElevatorPosition(0);
                    m_state = State.retractingElevator;
                }
                break;

            case retractingElevator:
                if (Math.abs(m_elevatorSubsystem.getEncoderPosition()) < ElevatorCalibrations.TOLERANCE) {
                    m_state = State.done;
                }
                break;

            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return m_state == State.done;
    }

}
