package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations.SwerveCalibrations;
import frc.robot.Constants.DriverConstants;
import frc.robot.lib.AccelerationLimiter;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The command for driving the robot.
 */
public class Drive extends CommandBase {

    private XboxController m_driver;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    private AccelerationLimiter m_strafeX;
    private AccelerationLimiter m_strafeY;

    private boolean m_robotRelative;

    /**
     * Input from the driver.
     *
     * @param driver              The driver of the robot.
     * @param drivetrainSubsystem The subsystem required for this command.
     */
    public Drive(XboxController driver, DrivetrainSubsystem drivetrainSubsystem) {
        m_driver = driver;
        m_drivetrainSubsystem = drivetrainSubsystem;

        m_strafeX = new AccelerationLimiter(SwerveCalibrations.MAX_ACCELERATION, SwerveCalibrations.MAX_DECELERATION);
        m_strafeY = new AccelerationLimiter(SwerveCalibrations.MAX_ACCELERATION, SwerveCalibrations.MAX_DECELERATION);

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_robotRelative = false;

        m_strafeX.reset(0);
        m_strafeY.reset(0);
    }

    @Override
    public void execute() {
        double strafeX = m_strafeX.calculate(MathUtil.applyDeadband(-m_driver.getLeftY(),
                DriverConstants.CONTROLLER_DEADBAND) * DriverConstants.MAX_STRAFE_SPEED);
        double strafeY = m_strafeY.calculate(MathUtil.applyDeadband(-m_driver.getLeftX(),
                DriverConstants.CONTROLLER_DEADBAND) * DriverConstants.MAX_STRAFE_SPEED);
        double rotate = MathUtil.applyDeadband(-m_driver.getRightX(), DriverConstants.CONTROLLER_DEADBAND)
                * DriverConstants.MAX_TURN_SPEED;

        m_drivetrainSubsystem.drive(strafeX, strafeY, rotate,
                !m_robotRelative);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, false);
    }
}