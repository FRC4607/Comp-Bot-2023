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

    private final CANSparkMax m_elevatorMotor;
    private final RelativeEncoder m_elevatorEncoder;

    private final DoubleLogEntry m_elevatorMotorPositionLog;
    private final DoubleLogEntry m_elevatorMotorVelocityLog;
    private final DoubleLogEntry m_elevatorMotorCurrentLog;
    private final DoubleLogEntry m_elevatorMotorVoltageLog;
    private final DoubleLogEntry m_elevatorMotorTempLog;

    /**
     * Declares and configures motors.
     */
    public ElevatorSubsystem() {
   
        m_elevatorMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_CANID, MotorType.kBrushless);
        
        m_elevatorMotor.restoreFactoryDefaults();
        m_elevatorMotor.setIdleMode(IdleMode.kBrake);
        m_elevatorMotor.setInverted(false);
        m_elevatorMotor.setSmartCurrentLimit(20, 40);
        m_elevatorEncoder = m_elevatorMotor.getEncoder();
        m_elevatorEncoder.setPositionConversionFactor(0);

        m_elevatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_elevatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_elevatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_elevatorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        
        DataLog log = DataLogManager.getLog();

        m_elevatorMotorPositionLog = new DoubleLogEntry(log, "Elevator Motor Position (deg)");
        m_elevatorMotorVelocityLog = new DoubleLogEntry(log, "Elevator Motor Velocity (deg)");
        m_elevatorMotorCurrentLog = new DoubleLogEntry(log, "Elevator Motor Current");
        m_elevatorMotorVoltageLog = new DoubleLogEntry(log, "Elevator Motor Voltage (volts)");
        m_elevatorMotorTempLog = new DoubleLogEntry(log, "Elevator Motor Temp (deg C)");

    }

    /**
     * Moves the Elevator.
     */
    public void moveElevator(double speed) {

        m_elevatorMotor.set(speed);
        
    }
}