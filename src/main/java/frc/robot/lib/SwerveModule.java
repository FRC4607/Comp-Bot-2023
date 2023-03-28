package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
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

    private SparkMaxAbsoluteEncoder m_turnAbsoluteEncoder;
    private RelativeEncoder m_turnRelativeEncoder;
    private RelativeEncoder m_driveEncoder;

    private SparkMaxPIDController m_turnPIDController;
    private SparkMaxPIDController m_drivePIDController;

    private SimpleMotorFeedforward m_driveFeedForward;

    private SwerveModuleState m_state;

    private double m_home;
    private double m_turnTarget = 0.0;
    private double m_driveTarget = 0.0;
    private double m_turnVoltage = 0.0;
    private double m_driveVoltage = 0.0;

    // ******* Logs *******

    private String m_label;
    private final DataLog m_log;

    private boolean m_pidTuning = false;
    private double m_kp;
    private double m_ki;
    private double m_kd;

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

    private final DoubleLogEntry m_driveMotorOutputLog;
    private final IntegerLogEntry m_driveMotorFaultsLog;
    private final DoubleLogEntry m_driveMotorSetpointLog;
    private final DoubleLogEntry m_driveMotorPositionLog;
    private final DoubleLogEntry m_driveMotorVelocityLog;
    private final DoubleLogEntry m_driveMotorCurrentLog;
    private final DoubleLogEntry m_driveMotorVoltageLog;
    private final DoubleLogEntry m_driveMotorTempLog;
    private final DoubleLogEntry m_driveCommandedVoltageLog;

    private final DoubleLogEntry m_turnMotorOutputLog;
    private final IntegerLogEntry m_turnMotorFaultsLog;
    private final DoubleLogEntry m_turnMotorSetpointLog;
    private final DoubleLogEntry m_turnMotorPositionLog;
    private final DoubleLogEntry m_turnMotorVelocityLog;
    private final DoubleLogEntry m_turnMotorCurrentLog;
    private final DoubleLogEntry m_turnMotorVoltageLog;
    private final DoubleLogEntry m_turnMotorTempLog;
    private final DoubleLogEntry m_turnCommandedVoltageLog;
    private final DoubleLogEntry m_turnMotorAbsoluteEncoderLog;

    private final DoubleLogEntry m_homeLog;

    /**
     * An encapsulates of a swerve module.
     *
     * @param driveMotorID The CAN ID of the drive motor
     * @param turnMotorID  The CAN ID of the turn motor
     * @param debug        Whether a value is sent to Shuffleboard
     */
    public SwerveModule(String label, int driveMotorID, int turnMotorID,
            boolean driverReversed, boolean debug) {

        m_label = label;

        m_home = Preferences.getDouble(m_label + ":home", 0.0);

        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setSmartCurrentLimit(80, 40);
        m_turnMotor.setSmartCurrentLimit(40, 40);

        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // Faults and Applied Output
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // Velocity, Bus Voltage, Temp, and Current
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Position
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // Faults and Applied Output
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // Velocity, Bus Voltage, Temp, and Current
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Positions
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // Duty Cycle Encoder Position
        m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // Duty Cycle Encoder Velocity

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
        m_turnPIDController.setSmartMotionMinOutputVelocity(
                SwerveCalibrations.TURN_MIN_VELOCITY * SwerveConstants.TURN_GEAR_RATIO / (2 * Math.PI), 0);

        m_drivePIDController.setP(SwerveCalibrations.DRIVE_KP);
        m_drivePIDController.setI(SwerveCalibrations.DRIVE_KI);
        m_drivePIDController.setD(SwerveCalibrations.DRIVE_KD);
        m_drivePIDController.setFF(SwerveCalibrations.DRIVE_KF);

        m_turnAbsoluteEncoder = m_turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_turnAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);

        m_driveFeedForward = new SimpleMotorFeedforward(SwerveCalibrations.DRIVE_FF_KS,
                SwerveCalibrations.DRIVE_FF_KV,
                SwerveCalibrations.DRIVE_FF_KA);

        if (m_pidTuning) {
            m_kp = SwerveCalibrations.DRIVE_KP;
            m_kd = SwerveCalibrations.DRIVE_KD;

            // PID Tunning
            SmartDashboard.putNumber("kP", m_kp);
            // SmartDashboard.putNumber("kI", SwerveCalibrations.TURN_KI);
            SmartDashboard.putNumber("kD", m_kd);
        }

        if (debug) {

            ShuffleboardLayout layout = Shuffleboard.getTab("Drivetrain").getLayout(label, "list");
            layout.addNumber("Home", this::getHome);
            layout.addNumber("Absolute Encoder", this::getAbsoluteEncoder);
            layout.addNumber("Turn Encoder", this::getTurnPos);
            layout.addNumber("Turn Target", this::getTurnTarget);
            layout.addNumber("Wheel Velocity", this::getWheelVelocity);
            layout.addNumber("Wheel Velocity Target", () -> {
                return m_driveTarget;
            });
            layout.addNumber("Wheel Velocity Error", () -> {
                return m_driveTarget - getWheelVelocity();
            });
            layout.addNumber("Wheel Position", m_driveEncoder::getPosition);
            layout.addNumber("Turn Error", () -> {
                return m_turnTarget - getTurnPos();
            });

        }

        m_log = DataLogManager.getLog();

        m_driveMotorOutputLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/output", m_label));
        m_driveMotorFaultsLog = new IntegerLogEntry(m_log, String.format("/swerve/%s/drive/faults", m_label));
        m_driveMotorSetpointLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/setpoint", m_label));
        m_driveMotorPositionLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/position", m_label));
        m_driveMotorVelocityLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/velocity", m_label));
        m_driveMotorCurrentLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/current", m_label));
        m_driveMotorVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/voltage", m_label));
        m_driveMotorTempLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/temp", m_label));
        m_driveCommandedVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/commandedVoltage", m_label));

        m_turnMotorOutputLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/output", m_label));
        m_turnMotorFaultsLog = new IntegerLogEntry(m_log, String.format("/swerve/%s/turn/faults", m_label));
        m_turnMotorSetpointLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/setpoint", m_label));
        m_turnMotorPositionLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/position", m_label));
        m_turnMotorVelocityLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/velocity", m_label));
        m_turnMotorCurrentLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/current", m_label));
        m_turnMotorVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/voltage", m_label));
        m_turnMotorTempLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/temp", m_label));
        m_turnCommandedVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/commandedVoltage", m_label));
        m_turnMotorAbsoluteEncoderLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/absolute", m_label));

        m_homeLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/home", m_label));

        logData();
        m_homeLog.append(m_home);

        m_state = new SwerveModuleState(0.0, Rotation2d.fromRadians(getTurnPos()));
    }

    // ******* Getters *******

    /**
     * Returns the position of the relative turn encoder in radian.
     */
    public double getTurnPos() {
        return m_turnRelativeEncoder.getPosition();
    }

    /**
     * Returns the absolute position of the turn encoder in radian.
     */
    public double getAbsoluteEncoder() {
        return m_turnAbsoluteEncoder.getPosition();
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
    public void setModuleState(SwerveModuleState state, boolean optimize) {
        if (optimize) {
            m_state = SwerveModuleState.optimize(state, new Rotation2d(getTurnPos()));
        } else {
            m_state = state;
        }

        double relativeEncoderValue = m_turnRelativeEncoder.getPosition();
        double target = m_state.angle.getRadians();

        // adds the closest int number of rotations to the target
        target += Math.round((relativeEncoderValue - target) / (2 * Math.PI)) * 2 * Math.PI;

        m_turnTarget = target;
        m_turnPIDController.setReference(target, ControlType.kPosition);

        m_driveTarget = m_state.speedMetersPerSecond;
        double ffVoltage = m_driveFeedForward.calculate(m_state.speedMetersPerSecond);

        m_drivePIDController.setReference(m_state.speedMetersPerSecond, ControlType.kVelocity, 0, ffVoltage,
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

    /**
     * Sets the voltage of the drive motor.
     *
     * @param voltage voltage the motor is set to
     */
    public void setDriveVoltage(double voltage) {
        m_driveVoltage = voltage;
        m_driveMotor.setVoltage(voltage);
    }

    /**
     * Sets the voltage of the turn motor.
     *
     * @param voltage voltage the motor is set to
     */
    public void setTurnVoltage(double voltage) {
        m_turnVoltage = voltage;
        m_turnMotor.setVoltage(voltage);
    }

    // ******* Logging *******

    /**
     * Logs the position, velocity, and targets of the swerve module.
     */
    private void logData() {
        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);
        
        m_driveMotorOutputLog.append(m_driveMotor.getAppliedOutput(), timeStamp);
        m_driveMotorFaultsLog.append(m_driveMotor.getFaults(), timeStamp);
        m_driveMotorSetpointLog.append(m_driveTarget, timeStamp);
        m_driveMotorPositionLog.append(m_driveEncoder.getPosition(), timeStamp);
        m_driveMotorVelocityLog.append(m_driveEncoder.getVelocity(), timeStamp);
        m_driveMotorCurrentLog.append(m_driveMotor.getOutputCurrent(), timeStamp);
        m_driveMotorVoltageLog.append(m_driveMotor.getBusVoltage(), timeStamp);
        m_driveMotorTempLog.append(m_driveMotor.getMotorTemperature(), timeStamp);
        m_driveCommandedVoltageLog.append(m_driveVoltage, timeStamp);

        m_turnMotorOutputLog.append(m_turnMotor.getAppliedOutput(), timeStamp);
        m_turnMotorFaultsLog.append(m_turnMotor.getFaults(), timeStamp);
        m_turnMotorSetpointLog.append(m_turnTarget, timeStamp);
        m_turnMotorPositionLog.append(m_turnRelativeEncoder.getPosition(), timeStamp);
        m_turnMotorVelocityLog.append(m_turnRelativeEncoder.getVelocity(), timeStamp);
        m_turnMotorCurrentLog.append(m_turnMotor.getOutputCurrent(), timeStamp);
        m_turnMotorVoltageLog.append(m_turnMotor.getBusVoltage(), timeStamp);
        m_turnMotorTempLog.append(m_turnMotor.getMotorTemperature(), timeStamp);
        m_turnCommandedVoltageLog.append(m_turnVoltage, timeStamp);

        m_turnMotorAbsoluteEncoderLog.append(m_turnAbsoluteEncoder.getPosition(), timeStamp);
    }

    /**
     * A function that should be called every loop.
     */
    public void update() {
        logData();

        if (m_pidTuning) {
            double kp = SmartDashboard.getNumber("kP",
                    SwerveCalibrations.TURN_KP);
            double ki = SmartDashboard.getNumber("kI",
                    SwerveCalibrations.TURN_KI);
            double kd = SmartDashboard.getNumber("kD",
                    SwerveCalibrations.TURN_KD);

            if (kp != m_kp) {
                m_drivePIDController.setP(kp);
                m_kp = kp;
            }
            if (ki != m_ki) {
                m_drivePIDController.setI(ki);
            }
            if (kd != m_kd) {
                m_drivePIDController.setD(kd);
                m_kd = kd;
            }
        }
    }
}
