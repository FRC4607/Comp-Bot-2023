package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * The command that moves the elevator.
 */
public class MoveElevator extends CommandBase {
    ElevatorSubsystem m_elevatorSubsystem;
    XboxController m_xboxController;

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
    public void execute() {
        double speed = -m_xboxController.getLeftTriggerAxis() + m_xboxController.getRightTriggerAxis();
        m_elevatorSubsystem.setSpeed(speed * 0.75);
    }
}