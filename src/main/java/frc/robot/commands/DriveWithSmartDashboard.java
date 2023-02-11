package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A Command to drive with Smart Dashboard.
 */
public class DriveWithSmartDashboard extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private SlewRateLimiter m_strafeX;
    private SlewRateLimiter m_strafeY;

    /**
     * A Command to drive with Smart Dashboard.
     *
     * @param drivetrainSubsystem The swerve Drivetrain Subsystem
     */
    public DriveWithSmartDashboard(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_strafeX = new SlewRateLimiter(5);
        m_strafeY = new SlewRateLimiter(5);

        addRequirements(m_drivetrainSubsystem);
        
        SmartDashboard.putNumber("Drive X", 0);
        SmartDashboard.putNumber("Drive Y", 0);
        SmartDashboard.putNumber("Drive Theta", 0);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double strafeX = m_strafeX.calculate(SmartDashboard.getNumber("Drive X", 0))
                * DriverConstants.MAX_STRAFE_SPEED;
        double strafeY = m_strafeY.calculate(SmartDashboard.getNumber("Drive Y", 0));
        double rotate = SmartDashboard.getNumber("Drive Theta", 0);

        m_drivetrainSubsystem.drive(strafeX, strafeY, rotate,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, false);
    }
}
