package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * The command that moves the arm.
 */
public class MoveElevatorSmartDashboard extends CommandBase {
    private ElevatorSubsystem m_elevatorSubsystem;

    /**
     * Defines what is necessary for this command.
     *
     * @param elevatorSubsystem The subsystem responsible for the arm.
     */
    public MoveElevatorSmartDashboard(ElevatorSubsystem elevatorSubsystem) {

        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);

    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Elevator Target Position", m_elevatorSubsystem.getEncoderPosition());
        m_elevatorSubsystem.resetController();
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setElevatorPosition(SmartDashboard.getNumber("Elevator Target Position", 0));
    }
}