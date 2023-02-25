package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * An Autonomous Command to place a Game Piece.
 */
public class PlaceGamePiece extends CommandBase {
    private PieceLevel m_pieceLevel;
    private ElevatorSubsystem m_elevatorSubsystem;
    private ArmSubsystem m_armSubsystem;
    private ManipulatorSubsystem m_manipulatorSubsystem;

    private State m_state;
    private int m_counter;

    /**
     * The level that the game piece is placed at.
     */
    public enum PieceLevel {
        TopCone(0),
        TopCube(1),
        MiddleCone(2),
        MiddleCube(3);

        public int m_value;

        PieceLevel(int value) {
            m_value = value;
        }
    }

    private enum State {
        extendingElevator,
        extendingArm,
        placingPiece,
        retractingArm,
        retractingElevator,
        done
    }

    /**
     * An Autonomous Command to place a Game Piece.
     *
     * @param pieceLevel The level and type of the game piece
     * @param elevatorSubsystem The Elevator Subsystem
     * @param armSubsystem the Arm Subsystem
     * @param manipulatorSubsystem the Manipulator Subsystem
     */
    public PlaceGamePiece(PieceLevel pieceLevel, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem,
            ManipulatorSubsystem manipulatorSubsystem) {
        m_pieceLevel = pieceLevel;
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;

        addRequirements(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    }

    @Override
    public void initialize() {
        m_state = State.extendingElevator;
        m_counter = 0;
        m_elevatorSubsystem
                .setElevatorPosition(ElevatorCalibrations.elevatorPlacementPositions()[m_pieceLevel.m_value]);
        m_elevatorSubsystem.resetController();
        m_armSubsystem.setArmTargetPosition(ArmCalibrations.POSITION_RETRACTED);
        m_armSubsystem.resetController();

    }

    @Override
    public void execute() {
        switch (m_state) {
            case extendingElevator:
                if (Math.abs(m_elevatorSubsystem.getEncoderPosition()
                        - ElevatorCalibrations.elevatorPlacementPositions()[m_pieceLevel.m_value])
                        < ElevatorCalibrations.TOLERANCE) {
                    m_armSubsystem.setArmTargetPosition(ArmCalibrations.armPlacementPositions()[m_pieceLevel.m_value]);
                    m_state = State.extendingArm;
                }
                break;

            case extendingArm:
                if (Math.abs(m_armSubsystem.getAbsoluteEncoderPosition()
                        - ArmCalibrations.armPlacementPositions()[m_pieceLevel.m_value]) < ArmCalibrations.TOLERANCE) {

                    // m_armSubsystem.setArmTargetPosition(ArmCalibrations.ARM_PLACEMENT_POSITIONS[m_pieceLevel.m_value]);
                    m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.OUTTAKE_SPEED);
                    m_state = State.placingPiece;
                }
                break;

            case placingPiece:
                if (m_counter > 25) {
                    m_armSubsystem.setArmTargetPosition(ArmCalibrations.POSITION_RETRACTED);
                    m_state = State.retractingArm;
                }
                m_counter++;
                break;

            case retractingArm:
                if (Math.abs(m_armSubsystem.getAbsoluteEncoderPosition()
                        - ArmCalibrations.POSITION_RETRACTED) < ArmCalibrations.TOLERANCE) {
                    m_elevatorSubsystem.setElevatorPosition(0);
                    m_manipulatorSubsystem.setSpeed(0);
                    m_state = State.retractingElevator;
                }
                break;

            case retractingElevator:
                if (Math.abs(m_elevatorSubsystem.getEncoderPosition() - 0) < ElevatorCalibrations.TOLERANCE) {
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
