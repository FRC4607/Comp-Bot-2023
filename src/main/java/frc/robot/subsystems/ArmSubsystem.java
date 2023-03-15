package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Constants.ArmConstants;

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
    private final DoubleLogEntry m_motorCommandedVoltageLog;
    private final BooleanLogEntry m_closedLoopLog;
    private final DoubleLogEntry m_trueGoalPositionLog;
    private final DoubleLogEntry m_goalPositionLog;
    private final DoubleLogEntry m_setpointPositionLog;
    private final DoubleLogEntry m_setpointVelocityLog;
    private final DoubleLogEntry m_feedforwardLog;
    private final DoubleLogEntry m_pidLog;
    private final StringLogEntry m_currentCommandLog;

    private final ProfiledPIDController m_pidController;
    private final ArmFeedforward m_feedforward;

    private double m_setpoint;
    private boolean m_closedLoop;
    private double m_commandedVoltage = 0.0;

    /**
     * Defines the wrist subsystem.
     */
    public ArmSubsystem() {

        m_motor = new CANSparkMax(ArmConstants.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(false);
        m_motor.setSmartCurrentLimit(40, 40);

        m_motorEncoder = m_motor.getEncoder();
        m_motorEncoder.setPositionConversionFactor(1.0);

        m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_absoluteEncoder.setPositionConversionFactor(360.0);
        m_absoluteEncoder.setVelocityConversionFactor(360.0);
        m_absoluteEncoder.setZeroOffset(MathUtil.inputModulus(-84 + 120, 0, 360));
        m_absoluteEncoder.setInverted(true);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // Faults and Applied Output
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 40); // Velocity, Bus Voltage, Temp, and Current
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40); // Position
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // Duty Cycle Position
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // Duty Cycle Velocity

        m_pidController = new ProfiledPIDController(ArmCalibrations.KP, ArmCalibrations.KI, ArmCalibrations.KD,
                new Constraints(ArmCalibrations.MAX_VELOCITY, ArmCalibrations.MAX_ACCELERATION));
        m_pidController.reset(getAbsoluteEncoderPosition());

        SmartDashboard.putData(m_pidController);

        m_feedforward = new ArmFeedforward(ArmCalibrations.KS, ArmCalibrations.KG, ArmCalibrations.KV,
                ArmCalibrations.KA);

        DataLog log = DataLogManager.getLog();

        m_motorOutputLog = new DoubleLogEntry(log, "/arm/motor/output");
        m_motorFaultsLog = new IntegerLogEntry(log, "/arm/motor/faults");
        m_motorPositionLog = new DoubleLogEntry(log, "/arm/motor/position");
        m_motorVelocityLog = new DoubleLogEntry(log, "/arm/motor/velocity");
        m_motorCurrentLog = new DoubleLogEntry(log, "/arm/motor/current");
        m_motorVoltageLog = new DoubleLogEntry(log, "/arm/motor/voltage");
        m_motorTempLog = new DoubleLogEntry(log, "/arm/motor/temp");
        m_motorCommandedVoltageLog = new DoubleLogEntry(log, "/arm/commandedVoltage");
        m_absolutePositionLog = new DoubleLogEntry(log, "/arm/absolute/position");
        m_absoluteVelocityLog = new DoubleLogEntry(log, "/arm/absolute/velocity");

        m_closedLoopLog = new BooleanLogEntry(log, "arm/closedLoop");
        m_trueGoalPositionLog = new DoubleLogEntry(log, "/arm/trueGoal");
        m_goalPositionLog = new DoubleLogEntry(log, "/arm/goal/position");
        m_setpointPositionLog = new DoubleLogEntry(log, "/arm/setpoint/position");
        m_setpointVelocityLog = new DoubleLogEntry(log, "/arm/setpoint/velocity");
        m_feedforwardLog = new DoubleLogEntry(log, "/arm/feedforward");
        m_pidLog = new DoubleLogEntry(log, "/arm/pid");

        m_currentCommandLog = new StringLogEntry(log, "/arm/command");


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

    public void setVoltage(double voltage) {
        m_commandedVoltage = voltage;
        m_motor.setVoltage(voltage);
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
        m_pidController.setGoal(m_setpoint - (3.75 + Math.sin(m_setpoint * Math.PI / 180.0) * 17.5));
        m_closedLoop = true;
    }

    public void resetController() {
        m_pidController.reset(getAbsoluteEncoderPosition());
    }

    @Override
    public void periodic() {

        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);
        
        double pid = m_pidController.calculate(getAbsoluteEncoderPosition());
        double ff = m_feedforward.calculate((m_pidController.getSetpoint().position - 90.0) * Math.PI / 180.0,
                m_pidController.getSetpoint().velocity);

        if (m_closedLoop) {
            m_commandedVoltage = pid + ff;
            m_motor.setVoltage(m_commandedVoltage);
        }


        m_motorOutputLog.append(m_motor.getAppliedOutput(), timeStamp);
        m_motorFaultsLog.append(m_motor.getFaults(), timeStamp);
        m_motorPositionLog.append(m_motorEncoder.getPosition(), timeStamp);
        m_motorVelocityLog.append(m_motorEncoder.getVelocity(), timeStamp);
        m_motorCurrentLog.append(m_motor.getOutputCurrent(), timeStamp);
        m_motorVoltageLog.append(m_motor.getBusVoltage(), timeStamp);
        m_motorTempLog.append(m_motor.getMotorTemperature(), timeStamp);
        m_motorCommandedVoltageLog.append(m_commandedVoltage, timeStamp);
        m_absolutePositionLog.append(getAbsoluteEncoderPosition(), timeStamp);
        m_absoluteVelocityLog.append(m_absoluteEncoder.getVelocity(), timeStamp);

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
