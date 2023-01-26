package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The command for driving the robot.
 */
public class Drive extends CommandBase {

    private XboxController m_driver;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    private SlewRateLimiter m_strafeX;
    private SlewRateLimiter m_strafeY;

    /**
     * Input from the driver.
     *
     * @param driver              The driver of the robot.
     * @param drivetrainSubsystem The subsystem required for this command.
     */
    public Drive(XboxController driver, DrivetrainSubsystem drivetrainSubsystem) {
        m_driver = driver;
        m_drivetrainSubsystem = drivetrainSubsystem;

        m_strafeX = new SlewRateLimiter(5);
        m_strafeY = new SlewRateLimiter(5);

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double strafeX = MathUtil.applyDeadband(m_strafeX.calculate(-m_driver.getLeftY()),
                DriverConstants.CONTROLLER_DEADBAND)
                * DriverConstants.MAX_STRAFE_SPEED;
        double strafeY = MathUtil.applyDeadband(m_strafeY.calculate(-m_driver.getLeftX()),
                DriverConstants.CONTROLLER_DEADBAND)
                * DriverConstants.MAX_STRAFE_SPEED;
        double rotate = MathUtil.applyDeadband(-m_driver.getRightX(), DriverConstants.CONTROLLER_DEADBAND)
                * DriverConstants.MAX_TURN_SPEED;

        m_drivetrainSubsystem.drive(strafeX, strafeY, rotate,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, false);
    }
}