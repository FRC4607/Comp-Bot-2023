package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * A command to move the arm to set position.
 */
public class MoveArmToPosition extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private double m_position;

    /**
     * A command to move the arm to set position.
     *
     * @param position The position the arm is told to go to.
     * @param armSubsystem The arm subsystem;
     */
    public MoveArmToPosition(double position, ArmSubsystem armSubsystem) {
        m_position = position;

        m_armSubsystem = armSubsystem;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.resetController();
        m_armSubsystem.setArmTargetPosition(m_position);
    }
}
