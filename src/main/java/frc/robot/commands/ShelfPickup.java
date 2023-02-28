package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command to collect game piece in auto. Not fully functional
 */
public class ShelfPickup extends CommandBase {
    private ElevatorSubsystem m_elevatorSubsystem;
    private ArmSubsystem m_armSubsystem;

    private State m_state;

    private enum State {
        extendingElevator,
        extendingArm,
        done
    }

    /**
     * A command to collect game piece in auto.
     *
     * @param elevatorSubsystem The Elevator Subsystem
     * @param armSubsystem The Arm Subsystem
     */
    public ShelfPickup(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;

        addRequirements(m_elevatorSubsystem, m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_state = State.extendingElevator;
        m_elevatorSubsystem.setElevatorPosition(ElevatorCalibrations.shelfPickup() + ElevatorCalibrations.TOLERANCE);
        m_elevatorSubsystem.resetController();
        m_armSubsystem.setArmTargetPosition(ArmCalibrations.POSITION_RETRACTED);
        m_armSubsystem.resetController();
    }

    @Override
    public void execute() {
        switch (m_state) {
            case extendingElevator:
                if (Math.abs(m_elevatorSubsystem.getEncoderPosition()
                        - ElevatorCalibrations.shelfPickup()) < ElevatorCalibrations.TOLERANCE) {

                    m_armSubsystem.setArmTargetPosition(ArmCalibrations.shelfPickup());
                    m_state = State.extendingArm;
                }
                break;

            case extendingArm:
                if (Math.abs(m_armSubsystem.getAbsoluteEncoderPosition()
                        - ArmCalibrations.shelfPickup()) < ArmCalibrations.TOLERANCE) {
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
