package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command to collect data for the arm FF.
 */
public class CalibrateElevatorFF extends CommandBase {
    
    
    private enum State {
        QuasiStaticUp,
        QuasiStaticDown,
        StepDown,
        StepUp,
        done
    }

    private final double m_quasiStaticRate = 0.25;
    private final double m_stepVoltage = 9.0;
    private final double m_downStartingVoltage = 1.0;
    
    private ElevatorSubsystem m_elevatorSubsystem;
    private State m_state;

    private double m_voltage;

    /**
     * A command to collect data for FF.
     *
     * @param elevatorSubsystem The elevator subsystem
     */
    public CalibrateElevatorFF(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;

        addRequirements(m_elevatorSubsystem);
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
                m_voltage += m_quasiStaticRate / 50;

                if (m_elevatorSubsystem.getEncoderPosition() >= ElevatorCalibrations.MAX_POSITION) {
                    m_state = State.QuasiStaticDown;
                    m_voltage = m_downStartingVoltage;
                }
                break;

            case QuasiStaticDown:
                m_voltage -= m_quasiStaticRate / 50;

                if (m_elevatorSubsystem.getEncoderPosition() <= 0.0) {
                    m_state = State.StepUp;
                    m_voltage = m_stepVoltage;
                }
                break;

            case StepUp:

                if (m_elevatorSubsystem.getEncoderPosition() >= ElevatorCalibrations.MAX_POSITION * 0.90) {
                    m_state = State.StepDown;
                    m_voltage = -m_stepVoltage;
                }
                

                break;

            case StepDown:

                if (m_elevatorSubsystem.getEncoderPosition() <= ElevatorCalibrations.MAX_POSITION * 0.1) {
                    m_state = State.done;
                    m_voltage = 0;
                }
                

                break;
        
            default:
                break;
        }

        m_elevatorSubsystem.setVoltage(m_voltage);
    }

    @Override
    public boolean isFinished() {
        return m_state == State.done;
    }
}