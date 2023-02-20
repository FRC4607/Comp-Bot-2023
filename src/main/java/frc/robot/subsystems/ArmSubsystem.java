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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Constants.ArmConstants;

/**
 * The subsystem responsible for the manipulator of the robot.
 */

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax m_armMotor;
    private final RelativeEncoder m_armEncoder;
    private final AbsoluteEncoder m_armAbsoluteEncoder;

    private final DoubleLogEntry m_armMotorOutputLog;
    private final IntegerLogEntry m_armMotorFaultsLog;
    private final DoubleLogEntry m_armMotorPositionLog;
    private final DoubleLogEntry m_armMotorVelocityLog;
    private final DoubleLogEntry m_armMotorCurrentLog;
    private final DoubleLogEntry m_armMotorVoltageLog;
    private final DoubleLogEntry m_armMotorTempLog;
    private final DoubleLogEntry m_armAbsolutePositionLog;
    private final DoubleLogEntry m_armAbsoluteVelocityLog;

    private final ProfiledPIDController m_pidController;

    private double m_setpoint;
    private boolean m_closedLoop;

    /**
     * Defines the wrist subsystem.
     */
    public ArmSubsystem() {

        m_armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_CAN_ID, MotorType.kBrushless);

        m_armMotor.restoreFactoryDefaults();
        m_armMotor.setIdleMode(IdleMode.kBrake);
        m_armMotor.setInverted(true);
        m_armMotor.setSmartCurrentLimit(40, 40);

        m_armEncoder = m_armMotor.getEncoder();
        m_armEncoder.setPositionConversionFactor(1.0);

        m_armAbsoluteEncoder = m_armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_armAbsoluteEncoder.setPositionConversionFactor(360);
        m_armAbsoluteEncoder.setVelocityConversionFactor(360 / 60);
        m_armAbsoluteEncoder.setZeroOffset(MathUtil.inputModulus(-105.0 + 120, 0, 360));
        m_armAbsoluteEncoder.setInverted(true);

        m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // Duty Cycle Position
        m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // Duty Cycle Velocity

        m_pidController = new ProfiledPIDController(ArmCalibrations.KP, ArmCalibrations.KI, ArmCalibrations.KD,
                new Constraints(ArmCalibrations.MAX_VELOCITY, ArmCalibrations.MAX_ACCELERATION));

        SmartDashboard.putData(m_pidController);

        DataLog log = DataLogManager.getLog();

        m_armMotorOutputLog = new DoubleLogEntry(log, "/arm/motor/output");
        m_armMotorFaultsLog = new IntegerLogEntry(log, "/arm/motor/faults");
        m_armMotorPositionLog = new DoubleLogEntry(log, "/arm/motor/position");
        m_armMotorVelocityLog = new DoubleLogEntry(log, "/arm/motor/velocity");
        m_armMotorCurrentLog = new DoubleLogEntry(log, "/arm/motor/current");
        m_armMotorVoltageLog = new DoubleLogEntry(log, "/arm/motor/voltage");
        m_armMotorTempLog = new DoubleLogEntry(log, "/arm/motor/temp");
        m_armAbsolutePositionLog = new DoubleLogEntry(log, "/arm/absolute/position");
        m_armAbsoluteVelocityLog = new DoubleLogEntry(log, "/arm/absolute/velocity");

    }

    /**
     * Sets the arm speed.
     *
     * @param speed The value passed in that sets the speed.
     */
    public void moveArm(double speed) {

        m_closedLoop = false;

        m_armMotor.set(speed);

    }

    /**
     * Gets the position of the absolute encoder in Degrees with Vertical being 0
     * and forward being positive.
     *
     * @return The position on degrees
     */
    public double getAbsoluteEncoderPosition() {
        return m_armAbsoluteEncoder.getPosition() - 120;
    }

    /**
     * Sets the target Arm Position.
     *
     * @param position The Target Arm Position in Degrees with Vertical being 0 and
     *                 forward being positive.
     */
    public void setArmTargetPosition(double position) {
        m_setpoint = position;
        m_closedLoop = true;
        m_pidController.setGoal(position);
    }

    @Override
    public void periodic() {

        if (m_closedLoop) {
            m_armMotor.setVoltage(m_pidController.calculate(getAbsoluteEncoderPosition())
                + Math.sin(getAbsoluteEncoderPosition() * Math.PI / 180) * ArmCalibrations.KG);
        }

        m_armMotorOutputLog.append(m_armMotor.getAppliedOutput());
        m_armMotorFaultsLog.append(m_armMotor.getFaults());
        m_armMotorPositionLog.append(m_armEncoder.getPosition());
        m_armMotorVelocityLog.append(m_armEncoder.getVelocity());
        m_armMotorCurrentLog.append(m_armMotor.getOutputCurrent());
        m_armMotorVoltageLog.append(m_armMotor.getBusVoltage());
        m_armMotorTempLog.append(m_armMotor.getMotorTemperature());
        m_armAbsolutePositionLog.append(m_armAbsoluteEncoder.getPosition());
        m_armAbsoluteVelocityLog.append(m_armAbsoluteEncoder.getPosition());

        SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Absolute Position", m_armAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("Arm Absolute Position 2", getAbsoluteEncoderPosition());

        m_armMotor.getFaults();

    }
}
