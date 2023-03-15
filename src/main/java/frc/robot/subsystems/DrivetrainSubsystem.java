package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.SwerveCalibrations;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.SwerveModule;
import java.util.ArrayList;

/**
 * Declares the swerve drivetrain as a subsystem.
 */
public class DrivetrainSubsystem extends SubsystemBase {

    private SwerveModule[] m_swerveModules;

    private Pigeon2 m_pigeon;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;

    private final DataLog m_log;

    private final DoubleLogEntry m_pigeonYawLog;
    private final DoubleLogEntry m_pigeonPitchLog;
    private final DoubleLogEntry m_pigeonRollLog;
    private final DoubleLogEntry m_calculatedPitch;
    private final StringLogEntry m_currentCommandLog;



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
                    SwerveConstants.DRIVE_ENCODER_REVERSED[i],
                    SwerveConstants.DEBUG);
        }

        m_pigeon = new Pigeon2(SwerveConstants.PIGEON2_CAN_ID);
        m_pigeon.setYaw(0);
        m_pigeon.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);

        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].homeEncoder();
        }

        m_kinematics = new SwerveDriveKinematics(SwerveConstants.POSITIONS);
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroYaw(), getModulePositions());

        m_log = DataLogManager.getLog();

        m_pigeonYawLog = new DoubleLogEntry(m_log, "swerve/pigeon/yaw");
        m_pigeonPitchLog = new DoubleLogEntry(m_log, "swerve/pigeon/pitch");
        m_pigeonRollLog = new DoubleLogEntry(m_log, "swerve/pigeon/roll");
        m_calculatedPitch = new DoubleLogEntry(m_log, "swerve/pigeon/calculated_pitch");
        m_currentCommandLog = new StringLogEntry(m_log, "/swerve/command");
    
        logData();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].update();
        }
        logData();

        m_odometry.update(getGyroYaw(), getModulePositions());

        // SmartDashboard.putNumber("Gyro Yaw (Deg)", getGyroRotation().getDegrees());
        // SmartDashboard.putNumber("Pigeon Yaw (Deg)", m_pigeon.getYaw());
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

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveCalibrations.MAX_SPEED_METER);

        for (int i = 0; i < swerveModuleStates.length; i++) {
            m_swerveModules[i].setModuleState(swerveModuleStates[i], true);
        }
    }

    /**
     * sends an error if an incorrect number of swerve modules are set up.
     *
     * @param swerveModuleStates to see if the correct # of modules are set up.
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates, boolean optimize) {

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveCalibrations.MAX_SPEED_METER);

        for (int i = 0; i < swerveModuleStates.length; i++) {
            m_swerveModules[i].setModuleState(swerveModuleStates[i], optimize);
        }
    }

    /**
     * The yaw of the gyro.
     *
     * @return The {@link Rotation2d} of the gyro's yaw
     */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    /**
     * The pitch of the gyro.
     *
     * @return The {@link Rotation2d} of the gyro's pitch
     */
    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(m_pigeon.getPitch());
    }

    /**
     * The roll of the gyro.
     *
     * @return The {@link Rotation2d} of the gyro's roll
     */
    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(m_pigeon.getRoll());
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
     * Gets the pitch of the robot regardless of the robot's yaw. For the gyro's raw yaw value, use {@code getGyroPitch}.

     * @return A {@link Rotation2d} of the robot's pitch relative to the field.
     */
    public Rotation2d getRobotPitch() {
        // This is probably not the correct way to do this, but it's a close enough approximation
        double roll = getGyroRoll().getRadians();
        double pitch = getGyroPitch().getRadians();
        double yaw = getPose().getRotation().getRadians();
        double theNumber = Math.cos(yaw) * pitch + Math.sin(yaw) * roll;
        return Rotation2d.fromRadians(theNumber);
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
        drive(strafeX, strafeY, rotate, fieldOrientated, true);
    }
    
    /**
     * Driver Input.
     *
     * @param strafeX         speed in m/s.
     * @param strafeY         speed in m/s.
     * @param rotate          rad/s
     * @param fieldOrientated whether or not to drive relative to the field.
     * @param autoX           if the modules should go to x when no input is detected
     */
    public void drive(double strafeX, double strafeY, double rotate, boolean fieldOrientated, boolean autoX) {
        ChassisSpeeds chassisSpeed;

        if (fieldOrientated) {
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(strafeX, strafeY, rotate,
                    m_odometry.getPoseMeters().getRotation());
        } else {
            chassisSpeed = new ChassisSpeeds(strafeX, strafeY, rotate);
        }
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeed);

        if (strafeX == 0.0 && strafeY == 0.0 && rotate == 0.0 && autoX) {
            moduleStates[0].angle = Rotation2d.fromDegrees(45);
            moduleStates[1].angle = Rotation2d.fromDegrees(135);
            moduleStates[2].angle = Rotation2d.fromDegrees(135);
            moduleStates[3].angle = Rotation2d.fromDegrees(45);
        }

        setModuleStates(moduleStates);

    }

    /**
     * Sets the module stated based off of the kinematics and chassis speeds
     * inputted.
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
     * Sets the voltage of the drive motors.
     *
     * @param voltage the voltage the motors are set to
     */
    public void setModuleDriveVoltage(double voltage) {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].setDriveVoltage(voltage);
        }
    }

    /**
     * Sets the voltage of the drive motors.
     *
     * @param voltage the voltage the motors are set to
     */
    public void setModuleTurnVoltage(double voltage) {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].setTurnVoltage(voltage);
        }
    }

    /**
     * Resets the robot's position on the field so that the current facing direction
     * is forward.
     */
    public void resetHeading() {
        Pose2d currentPose = getPose();
        Pose2d newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        m_odometry.resetPosition(getGyroYaw(), getModulePositions(), newPose);
    }

    /**
     * Sets the Odometry pose.
     *
     * @param pose The pose the odometry is set to.
     */
    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /**
     * A function that logs the data in the Drivetrain subsystem.
     */
    private void logData() {
        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);
        m_pigeonYawLog.append(m_pigeon.getYaw(), timeStamp);
        m_pigeonPitchLog.append(m_pigeon.getPitch(), timeStamp);
        m_pigeonRollLog.append(m_pigeon.getRoll(), timeStamp);
        m_calculatedPitch.append(getRobotPitch().getDegrees(), timeStamp);

        Command currentCommand = getCurrentCommand();
        m_currentCommandLog.append(currentCommand != null ? currentCommand.getName() : "None", timeStamp);
    }

}
