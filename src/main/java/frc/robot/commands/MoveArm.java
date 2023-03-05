package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.ArmSubsystem;

/**
 * The command that moves the arm.
 */
public class MoveArm extends CommandBase {
    ArmSubsystem m_armSubsystem;
    private XboxController m_operator;
    private double m_armPosition;

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
    public void initialize() {
        m_armPosition = m_armSubsystem.getAbsoluteEncoderPosition();
        m_armSubsystem.resetController();
    }

    @Override
    public void execute() {

        m_armPosition += MathUtil.applyDeadband(-m_operator.getLeftY(), DriverConstants.CONTROLLER_DEADBAND)
                * ArmCalibrations.ARM_OPERATOR_SPEED;

        m_armPosition = MathUtil.clamp(m_armPosition, ArmCalibrations.MIN_POSITION, ArmCalibrations.MAX_POSITION);
        m_armSubsystem.setArmTargetPosition(m_armPosition);
    }
}