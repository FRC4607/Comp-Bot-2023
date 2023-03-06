package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.subsystems.ArmSubsystem;

/**
 * A command to collect data for the arm FF data.
 */
public class CalibrateArmFF extends CommandBase {

    private enum State {
        MovingElevator,
        QuasiStaticUp,
        QuasiStaticDown,
        StepDown,
        StepUp,
        done
    }

    private final double m_quasiStaticRate = 0.25;
    private final double m_stepVoltage = 9.0;
    private final double m_downStartingVoltage = 0.5;

    private ArmSubsystem m_armSubsystem;
    private State m_state;

    private double m_voltage;

    /**
     * The constructor for a command to collect data for FF on the arm. The arm is
     * expected to start all the way down and the elevator extended out such that
     * it will not interfere.
     *
     * @param armSubsystem The arm subsystem
     */
    public CalibrateArmFF(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_state = State.QuasiStaticUp;
        m_voltage = 0.0;
    }

    @Override
    public void execute() {
        switch (m_state) {
            case QuasiStaticUp:
                m_voltage -= m_quasiStaticRate / 50;

                if (m_armSubsystem.getAbsoluteEncoderPosition() <= ArmCalibrations.MIN_POSITION) {
                    m_state = State.QuasiStaticDown;
                    m_voltage = m_downStartingVoltage;
                }
                break;

            case QuasiStaticDown:
                m_voltage += m_quasiStaticRate / 50;

                if (m_armSubsystem.getAbsoluteEncoderPosition() >= ArmCalibrations.MAX_POSITION) {
                    // if (m_quasiStaticUpReached > m_quasiStaticMax && m_quasiStaticDownReached >
                    // m_quasiStaticMax) {
                    // m_state = State.StepUp;
                    // m_elevatorSubsystem.setVoltage(m_stepVoltage);
                    // } else {
                    m_state = State.done;
                    m_voltage = -m_stepVoltage;
                    // }
                }
                break;

            case StepUp:

                if (m_armSubsystem
                        .getAbsoluteEncoderPosition() <= (ArmCalibrations.MAX_POSITION - ArmCalibrations.MIN_POSITION)
                                * 0.25 + ArmCalibrations.MIN_POSITION) {
                    m_state = State.StepDown;
                    m_voltage = m_stepVoltage;
                }

                break;

            case StepDown:

                if (m_armSubsystem
                        .getAbsoluteEncoderPosition() <= (ArmCalibrations.MAX_POSITION - ArmCalibrations.MIN_POSITION)
                                * 0.75 + ArmCalibrations.MIN_POSITION) {
                    m_state = State.done;
                    m_voltage = 0;
                }

                break;

            default:
                break;
        }

        m_armSubsystem.setVoltage(m_voltage);
    }

    @Override
    public boolean isFinished() {
        return m_state == State.done;
    }
}