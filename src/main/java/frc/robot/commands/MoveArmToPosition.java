package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

/**
 * A command to move the arm to set position.
 */
public class MoveArmToPosition extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private double m_position;
    private DoubleSupplier m_positionSupplier;

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

    /**
     * A command to move the arm to set position.
     *
     * @param positionSupplier The position supplies the arm is told to go to.
     * @param armSubsystem The arm subsystem;
     */
    public MoveArmToPosition(DoubleSupplier positionSupplier, ArmSubsystem armSubsystem) {
        m_positionSupplier = positionSupplier;

        m_armSubsystem = armSubsystem;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.resetController();
        if (m_positionSupplier != null) {
            m_position = m_positionSupplier.getAsDouble();
        }

        m_armSubsystem.setArmTargetPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_armSubsystem.getAbsoluteEncoderPosition() - m_position) < ArmCalibrations.TOLERANCE;
    }
}
