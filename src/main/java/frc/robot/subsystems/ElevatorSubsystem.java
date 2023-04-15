package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Constants.ElevatorConstants;

/**
 * The subsystem used for the elevator lift.
 */
public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax m_frontMotor;
    private final CANSparkMax m_backMotor;
    private final RelativeEncoder m_frontMotorEncoder;
    private final RelativeEncoder m_backMotorEncoder;
    private final DutyCycleEncoder m_absoluteEncoder;
    private final Encoder m_encoder;
    private final SparkMaxLimitSwitch m_forwardLimitSwitch;
    private final SparkMaxLimitSwitch m_reverseLimitSwitch;

    private final ProfiledPIDController m_pidController;
    private final ElevatorFeedforward m_ffController;

    private final DoubleLogEntry m_frontMotorOutputLog;
    private final IntegerLogEntry m_frontMotorFaultsLog;
    private final DoubleLogEntry m_frontMotorPositionLog;
    private final DoubleLogEntry m_frontMotorVelocityLog;
    private final DoubleLogEntry m_frontMotorCurrentLog;
    private final DoubleLogEntry m_frontMotorVoltageLog;
    private final DoubleLogEntry m_frontMotorTempLog;

    private final DoubleLogEntry m_backMotorOutputLog;
    private final IntegerLogEntry m_backMotorFaultsLog;
    private final DoubleLogEntry m_backMotorPositionLog;
    private final DoubleLogEntry m_backMotorVelocityLog;
    private final DoubleLogEntry m_backMotorCurrentLog;
    private final DoubleLogEntry m_backMotorVoltageLog;
    private final DoubleLogEntry m_backMotorTempLog;

    private final DoubleLogEntry m_commandedVoltageLog;

    private final DoubleLogEntry m_encoderPositionLog;
    private final DoubleLogEntry m_absoluteEncoderPositionLog;

    private final BooleanLogEntry m_closedLoopLog;
    private final DoubleLogEntry m_trueGoalPositionLog;
    private final DoubleLogEntry m_goalPositionLog;
    private final DoubleLogEntry m_setpointPositionLog;
    private final DoubleLogEntry m_setpointVelocityLog;
    private final DoubleLogEntry m_feedforwardLog;
    private final DoubleLogEntry m_pidLog;
    private final StringLogEntry m_currentCommandLog;

    private boolean m_closedLoop;
    private final Constraints m_downConstraints;
    private final Constraints m_upConstraints;

    private double m_commandedVoltage = 0.0;
    private double m_setpoint;

    /**
     * Declares and configures motors.
     */
    public ElevatorSubsystem() {

        m_frontMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_FRONT_MOTOR_CAN_ID, MotorType.kBrushless);
        m_backMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_BACK_MOTOR_CAN_ID, MotorType.kBrushless);

        m_frontMotor.restoreFactoryDefaults();
        m_frontMotor.setIdleMode(IdleMode.kBrake);
        m_frontMotor.setInverted(false);
        m_frontMotor.setSmartCurrentLimit(80, 60);

        m_backMotor.restoreFactoryDefaults();
        m_backMotor.setIdleMode(IdleMode.kBrake);
        m_backMotor.setInverted(false);
        m_backMotor.setSmartCurrentLimit(80, 60);

        m_backMotor.follow(m_frontMotor);

        m_frontMotorEncoder = m_frontMotor.getEncoder();
        m_frontMotorEncoder.setPositionConversionFactor(1.0 / ElevatorConstants.GEAR_RATIO_MOTOR);
        m_frontMotorEncoder.setVelocityConversionFactor(1.0 / 60.0 / ElevatorConstants.GEAR_RATIO_MOTOR);
        m_frontMotorEncoder.setPosition(0.0);

        m_backMotorEncoder = m_backMotor.getEncoder();
        m_backMotorEncoder.setPositionConversionFactor(1.0 / ElevatorConstants.GEAR_RATIO_MOTOR);
        m_backMotorEncoder.setVelocityConversionFactor(1.0 / 60.0 / ElevatorConstants.GEAR_RATIO_MOTOR);
        m_backMotorEncoder.setPosition(0.0);

        m_forwardLimitSwitch = m_frontMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_reverseLimitSwitch = m_frontMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimitSwitch.enableLimitSwitch(true);
        m_reverseLimitSwitch.enableLimitSwitch(false);

        m_upConstraints = new Constraints(ElevatorCalibrations.MAX_VELOCITY_UP,
                ElevatorCalibrations.MAX_ACCELERATION_UP);
        m_downConstraints = new Constraints(ElevatorCalibrations.MAX_VELOCITY_DOWN,
                ElevatorCalibrations.MAX_ACCELERATION_DOWN);

        m_pidController = new ProfiledPIDController(ElevatorCalibrations.KP, ElevatorCalibrations.KI,
                ElevatorCalibrations.KD, m_upConstraints);
        m_ffController = new ElevatorFeedforward(ElevatorCalibrations.KS, ElevatorCalibrations.KG,
                ElevatorCalibrations.KV, ElevatorCalibrations.KA);

        // SmartDashboard.putData(m_pidController);

        m_frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // Faults and Applied Output
        m_frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // Velocity, Bus Voltage, Temp, and Current
        m_frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Position
        m_frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        m_backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // Faults and Applied Output
        m_backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // Velocity, Bus Voltage, Temp, and Current
        m_backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Position
        m_backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        m_absoluteEncoder = new DutyCycleEncoder(ElevatorConstants.ABSOLUTE_ENCODER_PORT);
        m_absoluteEncoder.setDistancePerRotation(1.0 / ElevatorConstants.GEAR_RATIO_ENCODER);

        m_encoder = new Encoder(ElevatorConstants.ENCODER_A_PORT, ElevatorConstants.ENCODER_B_PORT);
        m_encoder.setDistancePerPulse(1.0 / 2048.0 / ElevatorConstants.GEAR_RATIO_ENCODER);
        m_encoder.setReverseDirection(true);

        DataLog log = DataLogManager.getLog();

        m_frontMotorOutputLog = new DoubleLogEntry(log, "/elevator/frontMotor/output");
        m_frontMotorFaultsLog = new IntegerLogEntry(log, "/elevator/frontMotor/faults");
        m_frontMotorPositionLog = new DoubleLogEntry(log, "/elevator/frontMotor/position");
        m_frontMotorVelocityLog = new DoubleLogEntry(log, "/elevator/frontMotor/velocity");
        m_frontMotorCurrentLog = new DoubleLogEntry(log, "/elevator/frontMotor/current");
        m_frontMotorVoltageLog = new DoubleLogEntry(log, "/elevator/frontMotor/voltage");
        m_frontMotorTempLog = new DoubleLogEntry(log, "/elevator/frontMotor/temp");

        m_backMotorOutputLog = new DoubleLogEntry(log, "/elevator/backMotor/output");
        m_backMotorFaultsLog = new IntegerLogEntry(log, "/elevator/backMotor/faults");
        m_backMotorPositionLog = new DoubleLogEntry(log, "/elevator/backMotor/position");
        m_backMotorVelocityLog = new DoubleLogEntry(log, "/elevator/backMotor/velocity");
        m_backMotorCurrentLog = new DoubleLogEntry(log, "/elevator/backMotor/current");
        m_backMotorVoltageLog = new DoubleLogEntry(log, "/elevator/backMotor/voltage");
        m_backMotorTempLog = new DoubleLogEntry(log, "/elevator/backMotor/temp");

        m_commandedVoltageLog = new DoubleLogEntry(log, "/elevator/commandedVoltage");
        m_encoderPositionLog = new DoubleLogEntry(log, "/elevator/encoder/position");
        m_absoluteEncoderPositionLog = new DoubleLogEntry(log, "/elevator/encoder/absolutePosition");

        m_closedLoopLog = new BooleanLogEntry(log, "elevator/closedLoop");
        m_trueGoalPositionLog = new DoubleLogEntry(log, "/elevator/trueGoal");
        m_goalPositionLog = new DoubleLogEntry(log, "/elevator/goal/position");
        m_setpointPositionLog = new DoubleLogEntry(log, "/elevator/setpoint/position");
        m_setpointVelocityLog = new DoubleLogEntry(log, "/elevator/setpoint/velocity");
        m_feedforwardLog = new DoubleLogEntry(log, "/elevator/feedforward");
        m_pidLog = new DoubleLogEntry(log, "/elevator/pid");

        m_currentCommandLog = new StringLogEntry(log, "/elevator/command");
    }

    /**
     * Sets the Elevator speed as a percentage.
     *
     * @param speed The percentage the elevator is set to
     */
    public void setSpeed(double speed) {
        m_closedLoop = false;
        m_frontMotor.set(speed);
    }

    /**
     * Sets the voltage of the elevator motor.
     *
     * @param voltage the voltage the motor is set to.
     */
    public void setVoltage(double voltage) {
        m_closedLoop = false;

        m_commandedVoltage = -voltage;
        m_frontMotor.setVoltage(-voltage);
    }

    public void setElevatorTargetPosition(double position) {
        setElevatorTargetPosition(position, false);
    }
    
    /**
     * Sets the position of the elevator with arm constraints accounted for.
     *
     * @param position The position the elevator is set to.
     */
    public void setElevatorTargetPosition(double position, boolean slow) {

        m_closedLoop = true;
        m_setpoint = position;

        if (slow) {
            m_pidController.setConstraints(new Constraints(2.0, 40.0));
        } else if (position < getEncoderPosition()) {
            m_pidController.setConstraints(m_downConstraints);
        } else {
            m_pidController.setConstraints(m_upConstraints);
        }

        m_pidController.setGoal(position);
    }

    public void resetController() {
        m_pidController.reset(getEncoderPosition());
    }

    public double getEncoderPosition() {
        // return m_encoder.getDistance();
        return m_frontMotorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Elevator Pos", getEncoderPosition());
        // SmartDashboard.putNumber("Elevator Voltage", m_commandedVoltage);

        double pid = m_pidController.calculate(getEncoderPosition());
        double ff = m_pidController.getSetpoint().position < 0.02 ? 0
                : m_ffController.calculate(m_pidController.getSetpoint().velocity, 0);

        if (m_closedLoop) {
            m_commandedVoltage = -(pid + ff);
            m_frontMotor.setVoltage(m_commandedVoltage);
        }

        if (m_forwardLimitSwitch.isPressed()) {
            m_encoder.reset();
            m_frontMotorEncoder.setPosition(0);
        }

        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);

        m_frontMotorOutputLog.append(m_frontMotor.getAppliedOutput(), timeStamp);
        m_frontMotorFaultsLog.append(m_frontMotor.getFaults(), timeStamp);
        m_frontMotorPositionLog.append(-m_frontMotorEncoder.getPosition(), timeStamp);
        m_frontMotorVelocityLog.append(-m_frontMotorEncoder.getVelocity(), timeStamp);
        m_frontMotorCurrentLog.append(m_frontMotor.getOutputCurrent(), timeStamp);
        m_frontMotorVoltageLog.append(m_frontMotor.getBusVoltage(), timeStamp);
        m_frontMotorTempLog.append(m_frontMotor.getMotorTemperature(), timeStamp);

        m_backMotorOutputLog.append(m_backMotor.getAppliedOutput(), timeStamp);
        m_backMotorFaultsLog.append(m_backMotor.getFaults(), timeStamp);
        m_backMotorPositionLog.append(-m_backMotorEncoder.getPosition(), timeStamp);
        m_backMotorVelocityLog.append(-m_backMotorEncoder.getVelocity(), timeStamp);
        m_backMotorCurrentLog.append(m_backMotor.getOutputCurrent(), timeStamp);
        m_backMotorVoltageLog.append(m_backMotor.getBusVoltage(), timeStamp);
        m_backMotorTempLog.append(m_backMotor.getMotorTemperature(), timeStamp);

        m_commandedVoltageLog.append(m_commandedVoltage, timeStamp);

        m_encoderPositionLog.append(m_encoder.getDistance(), timeStamp);
        m_absoluteEncoderPositionLog.append(m_absoluteEncoder.getDistance(), timeStamp);

        m_closedLoopLog.append(m_closedLoop, timeStamp);
        m_trueGoalPositionLog.append(m_setpoint, timeStamp);
        m_goalPositionLog.append(m_pidController.getGoal().position, timeStamp);
        m_setpointPositionLog.append(m_pidController.getSetpoint().position, timeStamp);
        m_setpointVelocityLog.append(m_pidController.getSetpoint().velocity, timeStamp);
        m_feedforwardLog.append(ff, timeStamp);
        m_pidLog.append(pid, timeStamp);

        Command currentCommand = getCurrentCommand();
        m_currentCommandLog.append(currentCommand != null ? currentCommand.getName() : "None", timeStamp);

    }

}