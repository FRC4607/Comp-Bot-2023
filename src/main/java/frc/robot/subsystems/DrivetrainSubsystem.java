package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.SwerveCalibrations;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.LimelightThread;
import frc.robot.lib.SwerveModule;
import java.util.ArrayList;
import java.util.List;

/**
 * Declares the swerve drivetrain as a subsystem.
 */
public class DrivetrainSubsystem extends SubsystemBase {

    private SwerveModule[] m_swerveModules;

    private Pigeon2 m_pigeon;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDrivePoseEstimator m_odometry;

    private boolean m_xMode = false;

    private final DataLog m_log;

    private final DoubleLogEntry m_pigeonYawLog;
    private final DoubleLogEntry m_pigeonPitchLog;
    private final DoubleLogEntry m_pigeonRollLog;
    private final DoubleLogEntry m_calculatedPitch;
    private final StringLogEntry m_currentCommandLog;
    private final DoubleArrayLogEntry m_odometryLog;
    private final DoubleArrayLogEntry m_pathLog;
    private final DoubleArrayLogEntry m_pathTargetLog;
    private final DoubleArrayLogEntry m_pathErrorLog;

    // private final LimelightThread m_llThread;
    // private final Thread m_threadObj;

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
        m_pigeon.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
        m_pigeon.setYaw(0);

        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].homeEncoder();
        }

        m_kinematics = new SwerveDriveKinematics(SwerveConstants.POSITIONS);
        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, getGyroYaw(), getModulePositions(), new Pose2d());

        m_log = DataLogManager.getLog();

        m_pigeonYawLog = new DoubleLogEntry(m_log, "swerve/pigeon/yaw");
        m_pigeonPitchLog = new DoubleLogEntry(m_log, "swerve/pigeon/pitch");
        m_pigeonRollLog = new DoubleLogEntry(m_log, "swerve/pigeon/roll");
        m_calculatedPitch = new DoubleLogEntry(m_log, "swerve/pigeon/calculated_pitch");
        m_currentCommandLog = new StringLogEntry(m_log, "/swerve/command");
        m_odometryLog = new DoubleArrayLogEntry(m_log, "/swerve/odometry");
        m_pathLog = new DoubleArrayLogEntry(m_log, "/swerve/path");
        m_pathTargetLog = new DoubleArrayLogEntry(m_log, "/swerve/targetPose");
        m_pathErrorLog = new DoubleArrayLogEntry(m_log, "/swerve/errorPose");

        PPSwerveControllerCommand.setLoggingCallbacks((path) -> {
            long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);

            List<double[]> states = path.getStates().stream().map((state) -> {
                return new double[] { state.poseMeters.getX(), state.poseMeters.getY(),
                        state.poseMeters.getRotation().getRadians() };
            }).toList();
            for (double[] state : states) {
                m_pathLog.append(state, timeStamp);
            }

        }, (pose) -> {
            m_pathTargetLog.append(new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
        }, null, (translation, rotation) -> {
            m_pathErrorLog.append(new double[] { translation.getX(), translation.getY(), rotation.getRadians() });
        });

        logData();

        // m_llThread = new LimelightThread(this);
        // m_threadObj = m_llThread.start();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].update();
        }
        logData();

        synchronized (m_odometry) {
            m_odometry.update(getGyroYaw(), getModulePositions());
        }

        SmartDashboard.putNumber("Calc Pitch", getRobotPitch().getDegrees());
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
        synchronized (m_odometry) {
            return m_odometry.getEstimatedPosition();
        }
    }

    /**
     * Gets the pitch of the robot regardless of the robot's yaw. For the gyro's raw
     * yaw value, use {@code getGyroPitch}.
     * 
     * @return A {@link Rotation2d} of the robot's pitch relative to the field.
     */
    public Rotation2d getRobotPitch() {
        // This is probably not the correct way to do this, but it's a close enough
        // approximation
        double roll = getGyroRoll().getRadians();
        double pitch = getGyroPitch().getRadians();
        double yaw = getPose().getRotation().getRadians();
        double theNumber = Math.cos(yaw) * pitch + Math.sin(yaw) * roll;
        return Rotation2d.fromRadians(theNumber);
    }

    public void toggleXMode() {
        m_xMode = !m_xMode;
    }

    public void setXMode(boolean xMode) {
        m_xMode = xMode;
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
        drive(strafeX, strafeY, rotate, fieldOrientated, m_xMode);
    }

    /**
     * Driver Input.
     *
     * @param strafeX         speed in m/s.
     * @param strafeY         speed in m/s.
     * @param rotate          rad/s
     * @param fieldOrientated whether or not to drive relative to the field.
     * @param autoX           if the modules should go to x when no input is
     *                        detected
     */
    public void drive(double strafeX, double strafeY, double rotate, boolean fieldOrientated, boolean autoX) {
        ChassisSpeeds chassisSpeed;

        if (fieldOrientated) {
            synchronized (m_odometry) {
                chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(strafeX, strafeY, rotate,
                        m_odometry.getEstimatedPosition().getRotation());
            }
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
        synchronized (m_odometry) {
            m_odometry.resetPosition(getGyroYaw(), getModulePositions(), newPose);
        }
    }

    /**
     * Sets the Odometry pose.
     *
     * @param pose The pose the odometry is set to.
     */
    public void setPose(Pose2d pose) {
        synchronized (m_odometry) {
            m_odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }
    }

    /**
     * A function that logs the data in the Drivetrain subsystem.
     */
    private void logData() {
        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);

        synchronized (m_odometry) {
            m_odometryLog.append(new double[] { m_odometry.getEstimatedPosition().getX(), m_odometry.getEstimatedPosition().getY(),
                    m_odometry.getEstimatedPosition().getRotation().getRadians() }, timeStamp);
            }

        m_pigeonYawLog.append(m_pigeon.getYaw(), timeStamp);
        m_pigeonPitchLog.append(m_pigeon.getPitch(), timeStamp);
        m_pigeonRollLog.append(m_pigeon.getRoll(), timeStamp);
        m_calculatedPitch.append(getRobotPitch().getDegrees(), timeStamp);

        Command currentCommand = getCurrentCommand();
        m_currentCommandLog.append(currentCommand != null ? currentCommand.getName() : "None", timeStamp);
    }

    public void synchronizedVisionUpdate(Pose2d newPose, double timestampSeconds) {
        synchronized (m_odometry) {
            m_odometry.addVisionMeasurement(newPose, timestampSeconds);
        }
    }
}
