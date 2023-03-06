package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * The command that moves the arm.
 */
public class MoveElevatorToPosition extends CommandBase {
    private ElevatorSubsystem m_elevatorSubsystem;
    private double m_position;

    /**
     * Defines what is necessary for this command.
     *
     * @param elevatorSubsystem The subsystem responsible for the arm.
     */
    public MoveElevatorToPosition(double position, ElevatorSubsystem elevatorSubsystem) {
        m_position = position;

        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);

    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.resetController();
        m_elevatorSubsystem.setElevatorTargetPosition(m_position);
    }
}