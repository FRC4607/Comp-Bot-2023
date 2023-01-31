package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Calibrations.SwerveCalibrations;
import frc.robot.Constants.SwerveConstants;

/**
 * A class that encapsulates a swerve module.
 */
public class SwerveModule {

    // ******* Control *******

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turnMotor;

    private DutyCycleEncoder m_turnAbsoluteEncoder;
    private RelativeEncoder m_turnRelativeEncoder;
    private RelativeEncoder m_driveEncoder;

    private SparkMaxPIDController m_turnPIDController;
    private SparkMaxPIDController m_drivePIDController;

    private SwerveModuleState m_state;

    private double m_home;
    private double m_turnTarget = 0.0;
    private double m_driveTarget = 0.0;

    // ******* Logs *******

    private String m_label;
    private final DataLog m_log;

    private boolean m_pidTuning = false;
    private double m_turnKP;
    private double m_turnKI;
    private double m_turnKD;

    /*
     * Logging:
     * SparkMax:
     * Motor Velocity - Frame 1
     * Motor Temp - Frame 1
     * Motor Voltage - Frame 1
     * Motor Current - Frame 1
     * Motor Position - Frame 2
     * 
     */

    private final DoubleLogEntry m_driveMotorSetpointLog;
    private final DoubleLogEntry m_driveMotorPositionLog;
    private final DoubleLogEntry m_driveMotorVelocityLog;
    private final DoubleLogEntry m_driveMotorCurrentLog;
    private final DoubleLogEntry m_driveMotorVoltageLog;
    private final DoubleLogEntry m_driveMotorTempLog;

    private final DoubleLogEntry m_turnMotorSetpointLog;
    private final DoubleLogEntry m_turnMotorPositionLog;
    private final DoubleLogEntry m_turnMotorVelocityLog;
    private final DoubleLogEntry m_turnMotorCurrentLog;
    private final DoubleLogEntry m_turnMotorVoltageLog;
    private final DoubleLogEntry m_turnMotorTempLog;

    private final DoubleLogEntry m_homeLog;
    private final DoubleLogEntry m_turnAbsoluteEncoderLog;

    /**
     * An encapsulates of a swerve module.
     *
     * @param driveMotorID The CAN ID of the drive motor
     * @param turnMotorID  The CAN ID of the turn motor
     * @param absEncoder   The roborio DIO port the absolute encoder is on
     * @param debug        Whether a value is sent to Shuffleboard
     */
    public SwerveModule(String label, int driveMotorID, int turnMotorID, int absEncoder,
            boolean driverReversed, boolean debug) {

        m_label = label;

        m_home = Preferences.getDouble(m_label + ":home", 0.0);

        m_turnAbsoluteEncoder = new DutyCycleEncoder(absEncoder);
        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setSmartCurrentLimit(20, 40);
        m_turnMotor.setSmartCurrentLimit(20, 40);

        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        m_turnMotor.clearFaults();
        m_driveMotor.clearFaults();

        m_drivePIDController = m_driveMotor.getPIDController();
        m_turnPIDController = m_turnMotor.getPIDController();

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnRelativeEncoder = m_turnMotor.getEncoder();

        m_driveEncoder.setPositionConversionFactor(
                SwerveConstants.WHEEL_CIRCUMFERENCE_METERS / SwerveConstants.DRIVE_GEAR_RATIO);
        m_driveEncoder.setVelocityConversionFactor(
                SwerveConstants.WHEEL_CIRCUMFERENCE_METERS / SwerveConstants.DRIVE_GEAR_RATIO / 60);

        m_driveMotor.setInverted(driverReversed);

        m_turnRelativeEncoder.setPositionConversionFactor(2 * Math.PI / SwerveConstants.TURN_GEAR_RATIO);
        m_turnRelativeEncoder.setVelocityConversionFactor(2 * Math.PI / SwerveConstants.TURN_GEAR_RATIO / 60);

        m_turnPIDController.setP(SwerveCalibrations.TURN_KP);
        m_turnPIDController.setI(SwerveCalibrations.TURN_KI);
        m_turnPIDController.setD(SwerveCalibrations.TURN_KD);
        m_turnPIDController.setSmartMotionMaxVelocity(SwerveCalibrations.TURN_MAX_VELOCITY, 0);
        m_turnPIDController.setSmartMotionMaxAccel(SwerveCalibrations.TURN_MAX_ACCELERATION, 0);
        m_turnPIDController.setSmartMotionMinOutputVelocity(SwerveCalibrations.TURN_MIN_VELOCITY, 0);

        m_drivePIDController.setP(SwerveCalibrations.DRIVE_KP);
        m_drivePIDController.setI(SwerveCalibrations.DRIVE_KI);
        m_drivePIDController.setD(SwerveCalibrations.DRIVE_KD);
        m_drivePIDController.setFF(SwerveCalibrations.DRIVE_KF);

        if (m_pidTuning) {
            m_turnKP = SwerveCalibrations.TURN_KP;
            m_turnKD = SwerveCalibrations.TURN_KD;

            // PID Tunning
            SmartDashboard.putNumber(label + " Turn kP", m_turnKP);
            SmartDashboard.putNumber(label + " Turn kI", SwerveCalibrations.TURN_KI);
            SmartDashboard.putNumber(label + " Turn kD", m_turnKD);
        }

        if (debug) {

            ShuffleboardLayout layout = Shuffleboard.getTab("Drivetrain").getLayout(label, "list");
            layout.addNumber("Home", this::getHome);
            layout.addNumber("Absolute Encoder", this::getAbsoluteEncoder);
            layout.addNumber("Turn Encoder", this::getTurnPos);
            layout.addNumber("Turn Target", this::getTurnTarget);
            layout.addNumber("Wheel Velocity", this::getWheelVelocity);
            layout.addNumber("Wheel Position", m_driveEncoder::getPosition);
            layout.addNumber("Turn Error", () -> {
                return m_turnTarget - getTurnPos();
            });

        }

        m_log = DataLogManager.getLog();

        m_driveMotorSetpointLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/setpoint", m_label));
        m_driveMotorPositionLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/position", m_label));
        m_driveMotorVelocityLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/velocity", m_label));
        m_driveMotorCurrentLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/current", m_label));
        m_driveMotorVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/voltage", m_label));
        m_driveMotorTempLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/temp", m_label));

        m_turnMotorSetpointLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/setpoint", m_label));
        m_turnMotorPositionLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/position", m_label));
        m_turnMotorVelocityLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/velocity", m_label));
        m_turnMotorCurrentLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/current", m_label));
        m_turnMotorVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/voltage", m_label));
        m_turnMotorTempLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/temp", m_label));

        m_homeLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/home", m_label));
        m_turnAbsoluteEncoderLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn_enc/absolute", m_label));

        logData();
        m_homeLog.append(m_home);
    }

    // ******* Getters *******

    /**
     * Returns the position of the relative turn encoder in radian.
     */
    public double getTurnPos() {
        return m_turnRelativeEncoder.getPosition();
        // m_turnRelativeEncoder.set
    }

    /**
     * Returns the absolute position of the turn encoder in radian.
     */
    public double getAbsoluteEncoder() {
        return m_turnAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI;
    }

    /**
     * Returns the home position in rad.
     *
     * @return home position in rad.
     */
    public double getHome() {
        return m_home;
    }

    /**
     * Returns the turn target in rad.
     *
     * @return turn target in rad.
     */
    public double getTurnTarget() {
        return m_turnTarget;
    }

    /**
     * Returns the drive wheel's velocity in m/s.
     *
     * @return drive wheel's velocity in m/s.
     */
    public double getWheelVelocity() {
        return m_driveEncoder.getVelocity();
    }

    /**
     * Returns the rotation of the drive wheel and the swerve module rotation.
     *
     * @return Drive wheel position and Swerve module rotation.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromRadians(getTurnPos()));
    }

    // ******* Setters *******

    /**
     * Sets the current position of the module as home.
     */
    public void setCurrentHome() {
        m_home = getAbsoluteEncoder();

        Preferences.setDouble(m_label + ":home", m_home);

        m_homeLog.append(m_home);

        homeEncoder();
    }

    /**
     * Sets the relative encoder based on the absolute encoder.
     */
    public void homeEncoder() {
        m_turnRelativeEncoder.setPosition(getAbsoluteEncoder() - m_home);
    }

    /**
     * Tells the turnPIDController to go to 0.
     */
    public void holdZero() {
        m_turnTarget = 0.0;
        m_turnPIDController.setReference(0, ControlType.kPosition);
    }

    /**
     * Sets the target module state.
     */
    public void setModuleState(SwerveModuleState state) {
        m_state = SwerveModuleState.optimize(state, new Rotation2d(getTurnPos()));

        double relativeEncoderValue = m_turnRelativeEncoder.getPosition();
        double target = m_state.angle.getRadians();

        // adds the closest int number of rotations to the target
        target += Math.round((relativeEncoderValue - target) / (2 * Math.PI)) * 2 * Math.PI;

        // TODO: Implement trapezoidal profile and Feed Forward
        m_turnTarget = target;
        m_turnPIDController.setReference(target, ControlType.kPosition);

        // TODO: Implement Feed Forward
        m_driveTarget = m_state.speedMetersPerSecond;
        m_drivePIDController.setReference(m_state.speedMetersPerSecond, ControlType.kVelocity, 0, 0.0,
                ArbFFUnits.kVoltage);
    }

    /**
     * Sets the idle mode.
     *
     * @param brakeMode sets idle mode true = brake false = coast.
     */
    public void setBrakeMode(boolean brakeMode) {
        m_driveMotor.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        m_turnMotor.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    // ******* Logging *******

    /**
     * Logs the position, velocity, and targets of the swerve module.
     */
    private void logData() {
        m_driveMotorSetpointLog.append(m_driveTarget);
        m_driveMotorPositionLog.append(m_driveEncoder.getPosition());
        m_driveMotorVelocityLog.append(m_driveEncoder.getVelocity());
        m_driveMotorCurrentLog.append(m_driveMotor.getOutputCurrent());
        m_driveMotorVoltageLog.append(m_driveMotor.getBusVoltage());
        m_driveMotorTempLog.append(m_driveMotor.getMotorTemperature());

        m_turnMotorSetpointLog.append(m_turnTarget);
        m_turnMotorPositionLog.append(m_turnRelativeEncoder.getPosition());
        m_turnMotorVelocityLog.append(m_turnRelativeEncoder.getVelocity());
        m_turnMotorCurrentLog.append(m_turnMotor.getOutputCurrent());
        m_turnMotorVoltageLog.append(m_turnMotor.getBusVoltage());
        m_turnMotorTempLog.append(m_turnMotor.getMotorTemperature());

        m_turnAbsoluteEncoderLog.append(m_turnAbsoluteEncoder.getDistance());
    }

    /**
     * A function that should be called every loop.
     */
    public void update() {
        logData();

        if (m_pidTuning) {
            double turnKP = SmartDashboard.getNumber(m_label + " Turn kP",
                    SwerveCalibrations.TURN_KP);
            double turnKI = SmartDashboard.getNumber(m_label + " Turn kI",
                    SwerveCalibrations.TURN_KI);
            double turnKD = SmartDashboard.getNumber(m_label + " Turn kD",
                    SwerveCalibrations.TURN_KD);

            if (turnKP != m_turnKP) {
                m_turnPIDController.setP(turnKP);
                m_turnKP = turnKP;
            }
            if (turnKI != m_turnKI) {
                m_turnPIDController.setI(turnKI);
                m_turnKI = turnKI;
            }
            if (turnKD != m_turnKD) {
                m_turnPIDController.setD(turnKD);
                m_turnKD = turnKD;
            }
        }
    }
}
