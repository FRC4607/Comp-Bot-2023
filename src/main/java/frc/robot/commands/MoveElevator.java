package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * The command that moves the elevator.
 */
public class MoveElevator extends CommandBase {
    ElevatorSubsystem m_elevatorSubsystem;
    XboxController m_xboxController;
    private double m_elevatorPosition;
    private boolean m_manual;

    /**
     * The command setup.
     *
     * @param xboxController    The device used for robot control.
     * @param elevatorSubsystem The subsystem that is the elevator.
     */
    public MoveElevator(XboxController xboxController, ElevatorSubsystem elevatorSubsystem) {
        m_xboxController = xboxController;

        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Open Elevator Control", false);
        m_manual = false;
        m_elevatorPosition = m_elevatorSubsystem.getEncoderPosition();
    }

    @Override
    public void execute() {
        double input = m_xboxController.getRightTriggerAxis() - m_xboxController.getLeftTriggerAxis();

        if (SmartDashboard.getBoolean("Open Elevator Control", true)) {
            if (!m_manual) {
                m_manual = true;
            }
            m_elevatorSubsystem.setSpeed(input);
        } else {
            if (m_manual) {
                m_manual = false;
                m_elevatorPosition = m_elevatorSubsystem.getEncoderPosition();
            }
            m_elevatorPosition += (input)
                    * ElevatorCalibrations.ELEVATOR_DRIVER_SPEED;

            m_elevatorPosition = m_elevatorPosition < 0 ? 0 : m_elevatorPosition;
            m_elevatorPosition = MathUtil.clamp(m_elevatorPosition, 0, ElevatorCalibrations.MAX_POSITION);

            m_elevatorSubsystem.setElevatorTargetPosition(m_elevatorPosition);
        }

    }
}