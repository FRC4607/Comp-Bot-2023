package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * The subsystem used for the elevator lift.
 */
public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax m_motorLeft;
    private final CANSparkMax m_motorRight;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    private final DoubleLogEntry m_leftMotorPositionLog;
    private final DoubleLogEntry m_leftMotorVelocityLog;
    private final DoubleLogEntry m_leftMotorCurrentLog;
    private final DoubleLogEntry m_leftMotorVoltageLog;
    private final DoubleLogEntry m_leftMotorTempLog;

    private final DoubleLogEntry m_rightMotorPositionLog;
    private final DoubleLogEntry m_rightMotorVelocityLog;
    private final DoubleLogEntry m_rightMotorCurrentLog;
    private final DoubleLogEntry m_rightMotorVoltageLog;
    private final DoubleLogEntry m_rightMotorTempLog;

    /**
     * Declares and configures motors.
     */
    public ElevatorSubsystem() {
   
        m_motorLeft = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_CANID, MotorType.kBrushless);
        m_motorRight = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_CANID, MotorType.kBrushless);
        
        m_motorLeft.restoreFactoryDefaults();
        m_motorLeft.setIdleMode(IdleMode.kBrake);
        m_motorLeft.setInverted(false);
        m_motorLeft.setSmartCurrentLimit(20, 40);
        m_leftEncoder = m_motorLeft.getEncoder();
        m_leftEncoder.setPositionConversionFactor(0);
        
        m_motorRight.restoreFactoryDefaults();
        m_motorRight.setIdleMode(IdleMode.kBrake);
        m_motorRight.setInverted(false);
        m_motorRight.setSmartCurrentLimit(20, 40);
        m_rightEncoder = m_motorRight.getEncoder();
        m_rightEncoder.setPositionConversionFactor(0);

        m_motorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_motorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_motorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_motorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_motorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_motorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_motorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        m_motorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        
        DataLog log = DataLogManager.getLog();

        m_leftMotorPositionLog = new DoubleLogEntry(log, "Left Motor Position (deg)");
        m_leftMotorVelocityLog = new DoubleLogEntry(log, "Left Motor Velocity (deg)");
        m_leftMotorCurrentLog = new DoubleLogEntry(log, "Left Motor Current");
        m_leftMotorVoltageLog = new DoubleLogEntry(log, "Left Motor Voltage (volts)");
        m_leftMotorTempLog = new DoubleLogEntry(log, "Left Motor Temp (deg C)");

        m_rightMotorPositionLog = new DoubleLogEntry(log, "Right Motor Position (deg)");
        m_rightMotorVelocityLog = new DoubleLogEntry(log, "Right Motor Velocity (deg)");
        m_rightMotorCurrentLog = new DoubleLogEntry(log, "Right Motor Current");
        m_rightMotorVoltageLog = new DoubleLogEntry(log, "Right Motor Voltage (volts)");
        m_rightMotorTempLog = new DoubleLogEntry(log, "Right Motor Temp (deg C)");

    }

    /**
     * Moves the Elevator.
     */
    public void moveElevator(double speed) {

        m_motorLeft.set(speed);
        m_motorRight.set(speed);
        
    }
}