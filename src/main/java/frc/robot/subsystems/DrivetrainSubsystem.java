package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.ADIS16470;
import frc.robot.lib.SwerveModule;
import java.util.ArrayList;

/**
 * Declares the swerve drivetrain as a subsystem.
 */
public class DrivetrainSubsystem extends SubsystemBase {
    
    private SwerveModule[] m_swerveModules;
    
    private ADIS16470 m_gyro;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;

    private final DataLog m_log;

    private final DoubleLogEntry m_gyroTempLog;
    private final DoubleLogEntry m_gyroYawLog;

    /**
     * Sets up the hardware used in the drivetrain.
     */
    public DrivetrainSubsystem() {

        m_swerveModules = new SwerveModule[SwerveConstants.LABELS.length];

        for (int i = 0; i < SwerveConstants.LABELS.length; i++) {
            m_swerveModules[i] = new SwerveModule(
                    SwerveConstants.LABELS[i],
                    SwerveConstants.DRIVE_CAN_IDS[i],
                    SwerveConstants.TURN_CAN_IDS[i],
                    SwerveConstants.ABS_ENCODER_DIO_PORT[i],
                    SwerveConstants.DRIVE_ENCODER_REVERSED[i],
                    SwerveConstants.DEBUG);
        }

        m_gyro = new ADIS16470();

        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].homeEncoder();
        }

        m_kinematics = new SwerveDriveKinematics(SwerveConstants.POSITIONS);
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroRotation(), getModulePositions());

        m_log = DataLogManager.getLog();

        m_gyroTempLog = new DoubleLogEntry(m_log, "swerve/gyro/temp");
        m_gyroYawLog = new DoubleLogEntry(m_log, "swerve/gyro/yaw");

        logData();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].update();
        }
        logData();

        m_odometry.update(getGyroRotation(), getModulePositions());

        SmartDashboard.putNumber("Gyro Yaw (Deg)", getGyroRotation().getDegrees());
    }
    
    /**
     * The Modules positions in an array.
     *
     * @return the array of all the modules positions
     */
    private SwerveModulePosition[] getModulePositions() {
        ArrayList<SwerveModulePosition> modulePositions = new ArrayList<>();

        for (SwerveModule swerveModule : m_swerveModules) {
            modulePositions.add(swerveModule.getPosition());
        }

        return (SwerveModulePosition[]) modulePositions.toArray(new SwerveModulePosition[m_swerveModules.length]);
    }

    /**
     * sends an error if an incorrect number of swerve modules are set up.
     *
     * @param swerveModuleStates to see if the correct # of modules are set up.
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED_METER_PER_SECOND);

        for (int i = 0; i < swerveModuleStates.length; i++) {
            m_swerveModules[i].setModuleState(swerveModuleStates[i]);
        }
    }

    /**
     * The rotation of the gyro.
     *
     * @return The {@link Rotation2d} of the gyro
     */
    public Rotation2d getGyroRotation() {
        
        return Rotation2d.fromDegrees(m_gyro.getAngle());

    }

    /**
     * The estimated Pose.
     *
     * @return The Pose estimation from the odometry
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Driver Input.
     *
     * @param strafeX         speed in m/s.
     * @param strafeY         speed in m/s.
     * @param rotate          rad/s
     * @param fieldOrientated whether or not to drive relative to the field.
     */
    public void drive(double strafeX, double strafeY, double rotate, boolean fieldOrientated) {
        ChassisSpeeds chassisSpeed;
        
        if (fieldOrientated) {
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(strafeX, strafeY, rotate,
                    m_odometry.getPoseMeters().getRotation());
        } else {
            chassisSpeed = new ChassisSpeeds(strafeX, strafeY, rotate);
        }
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeed);

        setModuleStates(moduleStates);

    }

    /**
     * Sets the module stated based off of the kinematics and chassis speeds inputted.
     *
     * @param chassisSpeeds The Target speeds of the Drivetrain
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStates(m_kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Sets the current Swerve Module Position to home.
     */
    public void setModuleHomes() {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].setCurrentHome();
        }
    }

    /**
     * Sets the type of idle mode.
     *
     * @param brakeMode whether or not to set the motors to hold position.
     */
    public void setBrakeMode(boolean brakeMode) {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].setBrakeMode(brakeMode);
        }
    }

    /**
     * Resets the robot's position on the field so that the current facing direction is forward.
     */
    public void resetHeading() {
        Pose2d currentPose = getPose();
        Pose2d newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        m_odometry.resetPosition(getGyroRotation(), getModulePositions(), newPose);
    }

    /**
     * Sets the Odometry pose.
     *
     * @param pose The pose the odometry is set to.
     */
    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    /**
     * A function that logs the data in the Drivetrain subsystem.
     */
    private void logData() {

        m_gyroTempLog.append(m_gyro.getTemp());
        m_gyroYawLog.append(m_gyro.getAngle());
    }

}