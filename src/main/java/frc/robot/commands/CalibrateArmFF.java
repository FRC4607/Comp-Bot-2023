package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

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
    private final double m_quasiStaticMax = 2.0;
    private final double m_stepVoltage = 3.0;

    private ArmSubsystem m_armSubsystem;
    private State m_state;
    private double m_quasiStaticUpReached;
    private double m_quasiStaticDownReached;

    private double m_voltage;
    private int m_counter;

    public CalibrateArmFF(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_state = State.QuasiStaticUp;
        m_quasiStaticUpReached = 0.0;
        m_quasiStaticDownReached = 0.5;
        m_voltage = 0.0;
    }

    @Override
    public void execute() {
        switch (m_state) {
            case QuasiStaticUp:
                m_voltage -= m_quasiStaticRate / 50;

                if (m_armSubsystem.getAbsoluteEncoderPosition() <= ArmCalibrations.MIN_POSITION) {
                    m_state = State.QuasiStaticDown;
                    m_quasiStaticUpReached = m_voltage;
                    m_voltage = m_quasiStaticDownReached;
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
                    m_quasiStaticDownReached = m_voltage;
                    m_voltage = -9;
                    // }
                }
                break;

            case StepUp:

                if (m_armSubsystem.getAbsoluteEncoderPosition()
                    <= (ArmCalibrations.MAX_POSITION - ArmCalibrations.MIN_POSITION) * 0.25 + ArmCalibrations.MIN_POSITION) {
                    m_state = State.StepDown;
                    m_voltage = 9;
                }

                break;

            case StepDown:

                if (m_armSubsystem.getAbsoluteEncoderPosition() 
                    <= (ArmCalibrations.MAX_POSITION - ArmCalibrations.MIN_POSITION) * 0.75 + ArmCalibrations.MIN_POSITION) {
                    m_state = State.StepDown;
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