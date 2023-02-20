package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.subsystems.ManipulatorSubsystem;
import java.util.function.BooleanSupplier;

/**
 * The class responsible for moving the manipulator.
 */
public class MoveManipulator extends CommandBase {

    ManipulatorSubsystem m_manipulatorSubsystem;
    BooleanSupplier m_operatorLeftBumper;
    BooleanSupplier m_operatorRightBumper;

    /**
     * The constructor that defines the input and requirements.
     *
     * @param operatorLeftBumper The Operator Left Bumper
     * @param operatorRightBumper The Operator Right Bumper
     * @param manipulatorSubsystem the manipulator subsystem.
     */
    public MoveManipulator(BooleanSupplier operatorLeftBumper, BooleanSupplier operatorRightBumper,
            ManipulatorSubsystem manipulatorSubsystem) {

        m_operatorLeftBumper = operatorLeftBumper;
        m_operatorRightBumper = operatorRightBumper;

        m_manipulatorSubsystem = manipulatorSubsystem;
        addRequirements(m_manipulatorSubsystem);

    }

    @Override
    public void execute() {

        if (m_operatorLeftBumper.getAsBoolean()) {
            m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.INTAKE_SPEED);
        } else if (m_operatorRightBumper.getAsBoolean()) {
            m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.OUTTAKE_SPEED);
        } else {
            m_manipulatorSubsystem.setSpeed(ManipulatorCalibrations.HOLD_SPEED);
        }

    }

}
