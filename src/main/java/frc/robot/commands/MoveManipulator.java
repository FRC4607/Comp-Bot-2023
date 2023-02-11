package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.MotorizedManipulator;
import java.util.function.BooleanSupplier;

/**
 * The class responsible for moving the manipulator.
 */
public class MoveManipulator extends CommandBase {

    MotorizedManipulator m_motorizedManipulator;
    BooleanSupplier m_dPadRight;
    BooleanSupplier m_dPadLeft;
    
    /**
     * The constructor that defines the input and requirements.
     *
     * @param dPadRight whether the right button of the dpad is pushed.
     * @param dPadLeft whether the left button of the dpad is pushed.
     * @param motorizedManipulator the manipulator subsystem.
     */
    public MoveManipulator(BooleanSupplier dPadRight, BooleanSupplier dPadLeft, MotorizedManipulator motorizedManipulator) {
    
        m_dPadRight = dPadRight;
        m_dPadLeft = dPadLeft;

        m_motorizedManipulator = motorizedManipulator;
        addRequirements(m_motorizedManipulator);

    }

    @Override
    public void execute() {

        if (m_dPadRight.getAsBoolean()) {

            m_motorizedManipulator.moveManipulator(ManipulatorConstants.MANIPULATOR_MOTOR_SPEED);

        } else if (m_dPadLeft.getAsBoolean()) {

            m_motorizedManipulator.moveManipulator(-(ManipulatorConstants.MANIPULATOR_MOTOR_SPEED));

        } else {

            m_motorizedManipulator.moveManipulator(0);

        }

    }
    
}
