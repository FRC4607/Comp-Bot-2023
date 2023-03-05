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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;

/**
 * The subsystem used for the elevator lift.
 */
public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor;
    private final RelativeEncoder m_motorEncoder;
    private final DutyCycleEncoder m_absoluteEncoder;
    private final Encoder m_encoder;
    private final SparkMaxLimitSwitch m_forwardLimitSwitch;
    private final SparkMaxLimitSwitch m_reverseLimitSwitch;

    private final ProfiledPIDController m_pidController;
    private final ElevatorFeedforward m_ffController;

    private final DoubleLogEntry m_motorOutputLog;
    private final IntegerLogEntry m_motorFaultsLog;
    private final DoubleLogEntry m_motorPositionLog;
    private final DoubleLogEntry m_motorVelocityLog;
    private final DoubleLogEntry m_motorCurrentLog;
    private final DoubleLogEntry m_motorVoltageLog;
    private final DoubleLogEntry m_motorTempLog;
    private final DoubleLogEntry m_motorCommandedVoltageLog;
    private final DoubleLogEntry m_encoderPositionLog;
    private final DoubleLogEntry m_absoluteEncoderPositionLog;

    private boolean m_closedLoop;

    private double m_commandedVoltage = 0.0;

    /**
     * Declares and configures motors.
     */
    public ElevatorSubsystem() {

        m_motor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(false);
        m_motor.setSmartCurrentLimit(80, 40);

        m_motorEncoder = m_motor.getEncoder();
        m_motorEncoder.setPositionConversionFactor(1.0 / ElevatorConstants.GEAR_RATIO_MOTOR);
        m_motorEncoder.setVelocityConversionFactor(1.0 / 60.0 / ElevatorConstants.GEAR_RATIO_MOTOR);
        m_motorEncoder.setPosition(0.0);

        m_forwardLimitSwitch = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_reverseLimitSwitch = m_motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimitSwitch.enableLimitSwitch(true);
        m_reverseLimitSwitch.enableLimitSwitch(false);

        m_pidController = new ProfiledPIDController(ElevatorCalibrations.KP, ElevatorCalibrations.KI,
                ElevatorCalibrations.KD,
                new Constraints(ElevatorCalibrations.MAX_VELOCITY, ElevatorCalibrations.MAX_ACCELERATION));
        m_ffController = new ElevatorFeedforward(ElevatorCalibrations.KS, ElevatorCalibrations.KG,
                ElevatorCalibrations.KV, ElevatorCalibrations.KA);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // Faults and Applied Output
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10); // Velocity, Bus Voltage, Temp, and Current
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10); // Position
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        m_absoluteEncoder = new DutyCycleEncoder(ElevatorConstants.ABSOLUTE_ENCODER_PORT);
        m_encoder = new Encoder(ElevatorConstants.ENCODER_A_PORT, ElevatorConstants.ENCODER_B_PORT);

        m_encoder.setDistancePerPulse(Math.PI * 2 / 2048);
        m_encoder.setReverseDirection(true);
        DataLog log = DataLogManager.getLog();

        m_motorOutputLog = new DoubleLogEntry(log, "/elevator/motor/output");
        m_motorFaultsLog = new IntegerLogEntry(log, "/elevator/motor/faults");
        m_motorPositionLog = new DoubleLogEntry(log, "/elevator/motor/position");
        m_motorVelocityLog = new DoubleLogEntry(log, "/elevator/motor/velocity");
        m_motorCurrentLog = new DoubleLogEntry(log, "/elevator/motor/current");
        m_motorVoltageLog = new DoubleLogEntry(log, "/elevator/motor/voltage");
        m_motorTempLog = new DoubleLogEntry(log, "/elevator/motor/temp");
        m_motorCommandedVoltageLog = new DoubleLogEntry(log, "/elevator/commandedVoltage");
        m_encoderPositionLog = new DoubleLogEntry(log, "/elevator/encoder/position");
        m_absoluteEncoderPositionLog = new DoubleLogEntry(log, "/elevator/encoder/absolutePosition");

    }

    /**
     * Sets the Elevator speed as a percentage.
     *
     * @param speed The percentage the elevator is set to
     */
    public void setSpeed(double speed) {
        m_closedLoop = false;
        m_motor.set(speed);
    }

    /**
     * Sets the voltage of the elevator motor.
     *
     * @param voltage the voltage the motor is set to.
     */
    public void setVoltage(double voltage) {
        m_closedLoop = false;

        m_commandedVoltage = -voltage;
        m_motor.setVoltage(-voltage);
    }

    /**
     * Sets the position of the elevator with arm constraints accounted for.
     *
     * @param position The position the elevator is set to.
     */
    public void setElevatorPosition(double position) {
        m_closedLoop = true;
        // if (position < ElevatorCalibrations.ARM_CLEARANCE &&
        // RobotContainer.getInstance().m_armSubsystem
        // .getAbsoluteEncoderPosition() > ArmCalibrations.ELEVATOR_CLEARANCE) {
        // m_pidController.setGoal(ElevatorCalibrations.ARM_CLEARANCE);
        // } else {
        m_pidController.setGoal(position);
        // }
    }

    public void resetController() {
        m_pidController.reset(getEncoderPosition());
    }

    public double getEncoderPosition() {
        // return m_encoder.getDistance();
        return -m_motorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        if (m_closedLoop) {
            double pid = m_pidController.calculate(getEncoderPosition());
            double ff = m_ffController.calculate(m_pidController.getSetpoint().velocity, 0);
            m_commandedVoltage = -(pid + ff);
            m_motor.setVoltage(m_commandedVoltage);
        }

        if (m_forwardLimitSwitch.isPressed()) {
            m_encoder.reset();
            m_motorEncoder.setPosition(0);
        }

        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);

        m_motorOutputLog.append(m_motor.getAppliedOutput(), timeStamp);
        m_motorFaultsLog.append(m_motor.getFaults(), timeStamp);
        m_motorPositionLog.append(-m_motorEncoder.getPosition(), timeStamp);
        m_motorVelocityLog.append(-m_motorEncoder.getVelocity(), timeStamp);
        m_motorCurrentLog.append(m_motor.getOutputCurrent(), timeStamp);
        m_motorVoltageLog.append(m_motor.getBusVoltage(), timeStamp);
        m_motorTempLog.append(m_motor.getMotorTemperature(), timeStamp);
        m_motorCommandedVoltageLog.append(m_commandedVoltage, timeStamp);
        // m_encoderPositionLog.append(m_encoder.getPosition(), timeStamp);
        // m_absoluteEncoderPositionLog.append(m_absoluteEncoder.getDistance(),
        // timeStamp);

        m_motor.getFaults();

        SmartDashboard.putNumber("Elevator Position Internal", -m_motorEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Velocity Internal", -m_motorEncoder.getVelocity());
        SmartDashboard.putNumber("Elevator Current", -m_motor.getOutputCurrent());
    }

}