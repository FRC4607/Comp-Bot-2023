package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * The command that moves the arm.
 */
public class MoveArmSmartDashboard extends CommandBase {
    ArmSubsystem m_armSubsystem;

    /**
     * Defines what is necessary for this command.
     *
     * @param armSubsystem The subsystem responsible for the arm.
     */
    public MoveArmSmartDashboard(ArmSubsystem armSubsystem) {

        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);

    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Arm Setpoint", m_armSubsystem.getAbsoluteEncoderPosition());
        m_armSubsystem.resetController();
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmTargetPosition(
                SmartDashboard.getNumber("Arm Setpoint", m_armSubsystem.getAbsoluteEncoderPosition()));
    }
}