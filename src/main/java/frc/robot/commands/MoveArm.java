package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.ArmSubsystem;

/**
 * The command that moves the arm.
 */
public class MoveArm extends CommandBase {
    ArmSubsystem m_armSubsystem;
    private XboxController m_operator;

    /**
     * Defines what is necessary for this command.
     *
     * @param operator     The operator controller
     * @param armSubsystem The subsystem responsible for the arm.
     */
    public MoveArm(XboxController operator, ArmSubsystem armSubsystem) {
        m_operator = operator;
        m_armSubsystem = armSubsystem;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void execute() {
        m_armSubsystem.moveArm(MathUtil.applyDeadband(-m_operator.getLeftY(), DriverConstants.CONTROLLER_DEADBAND));
    }
}