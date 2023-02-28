package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

/**
 * The subsystem responsible for the manipulator of the robot.
 */

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor;
    private final RelativeEncoder m_motorEncoder;
    private final AbsoluteEncoder m_absoluteEncoder;

    private final DoubleLogEntry m_motorOutputLog;
    private final IntegerLogEntry m_motorFaultsLog;
    private final DoubleLogEntry m_motorPositionLog;
    private final DoubleLogEntry m_motorVelocityLog;
    private final DoubleLogEntry m_motorCurrentLog;
    private final DoubleLogEntry m_motorVoltageLog;
    private final DoubleLogEntry m_motorTempLog;
    private final DoubleLogEntry m_absolutePositionLog;
    private final DoubleLogEntry m_absoluteVelocityLog;

    private final ProfiledPIDController m_pidController;

    private double m_setpoint;
    private boolean m_closedLoop;

    /**
     * Defines the wrist subsystem.
     */
    public ArmSubsystem() {

        m_motor = new CANSparkMax(ArmConstants.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(40, 40);

        m_motorEncoder = m_motor.getEncoder();
        m_motorEncoder.setPositionConversionFactor(1.0);

        m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_absoluteEncoder.setPositionConversionFactor(360);
        m_absoluteEncoder.setVelocityConversionFactor(360 / 60);
        m_absoluteEncoder.setZeroOffset(MathUtil.inputModulus(-105.0 + 120, 0, 360));
        m_absoluteEncoder.setInverted(true);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // Duty Cycle Position
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // Duty Cycle Velocity

        m_pidController = new ProfiledPIDController(ArmCalibrations.KP, ArmCalibrations.KI, ArmCalibrations.KD,
                new Constraints(ArmCalibrations.MAX_VELOCITY, ArmCalibrations.MAX_ACCELERATION));
        m_pidController.reset(getAbsoluteEncoderPosition());

        DataLog log = DataLogManager.getLog();

        m_motorOutputLog = new DoubleLogEntry(log, "/arm/motor/output");
        m_motorFaultsLog = new IntegerLogEntry(log, "/arm/motor/faults");
        m_motorPositionLog = new DoubleLogEntry(log, "/arm/motor/position");
        m_motorVelocityLog = new DoubleLogEntry(log, "/arm/motor/velocity");
        m_motorCurrentLog = new DoubleLogEntry(log, "/arm/motor/current");
        m_motorVoltageLog = new DoubleLogEntry(log, "/arm/motor/voltage");
        m_motorTempLog = new DoubleLogEntry(log, "/arm/motor/temp");
        m_absolutePositionLog = new DoubleLogEntry(log, "/arm/absolute/position");
        m_absoluteVelocityLog = new DoubleLogEntry(log, "/arm/absolute/velocity");

    }

    /**
     * Sets the arm speed.
     *
     * @param speed The value passed in that sets the speed.
     */
    public void moveArm(double speed) {
        m_closedLoop = false;
        m_motor.set(speed);
    }

    /**
     * Gets the position of the absolute encoder in Degrees with Vertical being 0
     * and forward being positive.
     *
     * @return The position on degrees
     */
    public double getAbsoluteEncoderPosition() {
        return m_absoluteEncoder.getPosition() - 120;
    }

    /**
     * Sets the target Arm Position.
     *
     * @param position The Target Arm Position in Degrees with Vertical being 0 and
     *                 forward being positive.
     */
    public void setArmTargetPosition(double position) {
        m_setpoint = MathUtil.clamp(position, ArmCalibrations.MIN_POSITION, ArmCalibrations.MAX_POSITION);

        if (m_setpoint > ArmCalibrations.ELEVATOR_CLEARANCE && RobotContainer.getInstance().m_elevatorSubsystem
                .getEncoderPosition() < ElevatorCalibrations.ARM_CLEARANCE) {
            m_pidController.setGoal(ArmCalibrations.ELEVATOR_CLEARANCE);
        } else {
            m_pidController.setGoal(m_setpoint);
        }
        m_closedLoop = true;
    }

    public void resetController() {
        m_pidController.reset(getAbsoluteEncoderPosition());
    }

    @Override
    public void periodic() {

        if (m_closedLoop) {
            m_motor.setVoltage(m_pidController.calculate(getAbsoluteEncoderPosition())
                + Math.sin(getAbsoluteEncoderPosition() * Math.PI / 180) * ArmCalibrations.KG);
        }

        m_motorOutputLog.append(m_motor.getAppliedOutput());
        m_motorFaultsLog.append(m_motor.getFaults());
        m_motorPositionLog.append(m_motorEncoder.getPosition());
        m_motorVelocityLog.append(m_motorEncoder.getVelocity());
        m_motorCurrentLog.append(m_motor.getOutputCurrent());
        m_motorVoltageLog.append(m_motor.getBusVoltage());
        m_motorTempLog.append(m_motor.getMotorTemperature());
        m_absolutePositionLog.append(m_absoluteEncoder.getPosition());
        m_absoluteVelocityLog.append(m_absoluteEncoder.getPosition());

        m_motor.getFaults();

    }
}
