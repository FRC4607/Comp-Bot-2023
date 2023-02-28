package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command to collect game piece in auto. Not fully functional
 */
public class Retract extends CommandBase {
    private ElevatorSubsystem m_elevatorSubsystem;
    private ArmSubsystem m_armSubsystem;

    private State m_state;

    private enum State {
        retractingArm,
        retractingElevator,
        done
    }

    /**
     * A command to collect game piece in auto.
     *
     * @param elevatorSubsystem The Elevator Subsystem
     * @param armSubsystem      The Arm Subsystem
     */
    public Retract(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;

        addRequirements(m_elevatorSubsystem, m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_state = State.retractingArm;
        m_elevatorSubsystem.resetController();
        m_armSubsystem.setArmTargetPosition(ArmCalibrations.POSITION_RETRACTED);
        m_armSubsystem.resetController();
    }

    @Override
    public void execute() {
        switch (m_state) {
            case retractingArm:
                if (Math.abs(m_armSubsystem.getAbsoluteEncoderPosition()
                        - ArmCalibrations.POSITION_RETRACTED) < ArmCalibrations.TOLERANCE) {
                    m_elevatorSubsystem.setElevatorPosition(0);
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
