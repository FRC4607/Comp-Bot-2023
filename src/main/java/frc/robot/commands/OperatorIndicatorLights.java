package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.IndicatorSubsystem;
import frc.robot.subsystems.IndicatorSubsystem.PieceIndicatorState;

/**
 * 
 */
public class OperatorIndicatorLights extends CommandBase{
    private POVButton m_operatorDPadUp;
    private POVButton m_operatorDPadDown;
    private IndicatorSubsystem m_indicatorSubsystem;

    /**
     * 
     * @param operator
     * @param indicatorSubsystem
     */
    public OperatorIndicatorLights(XboxController operator, IndicatorSubsystem indicatorSubsystem) {
        m_operatorDPadUp = new POVButton(operator, 0);
        m_operatorDPadDown = new POVButton(operator, 180);
        m_indicatorSubsystem = indicatorSubsystem;

        addRequirements(m_indicatorSubsystem);
    }

    @Override
    public void execute() {
        if (m_operatorDPadUp.getAsBoolean()) {
            m_indicatorSubsystem.setIndicator(PieceIndicatorState.CONE);
        } else if (m_operatorDPadDown.getAsBoolean()) {
            m_indicatorSubsystem.setIndicator(PieceIndicatorState.CUBE);
        } else {
            m_indicatorSubsystem.setIndicator(PieceIndicatorState.NONE);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
