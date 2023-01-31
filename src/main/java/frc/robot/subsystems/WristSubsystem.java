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
import frc.robot.Constants.ManipulatorConstants;

/**
 * The subsystem responsible for the manipulator of the robot.
 */
public class WristSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_motorWrist;
    private final RelativeEncoder m_wristEncoder;

    private final DoubleLogEntry m_wristPositionLog;
    private final DoubleLogEntry m_wristVelocityLog;
    private final DoubleLogEntry m_wristCurrentLog;
    private final DoubleLogEntry m_wristVoltageLog;
    private final DoubleLogEntry m_wristTempLog;

    /**
     * Defines the wrist subsystem.
     */
    public WristSubsystem() {
   
        m_motorWrist = new CANSparkMax(ManipulatorConstants.WRIST_MOTOR_CANID, MotorType.kBrushless);
        
        m_motorWrist.restoreFactoryDefaults();
        m_motorWrist.setIdleMode(IdleMode.kBrake);
        m_motorWrist.setInverted(false);
        m_motorWrist.setSmartCurrentLimit(10, 20);
        m_wristEncoder = m_motorWrist.getEncoder();
        m_wristEncoder.setPositionConversionFactor(0);

        m_motorWrist.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_motorWrist.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_motorWrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_motorWrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        
        DataLog log = DataLogManager.getLog();

        m_wristPositionLog = new DoubleLogEntry(log, "Wrist Motor Position (deg)");
        m_wristVelocityLog = new DoubleLogEntry(log, "Wrist Motor Velocity (deg)");
        m_wristCurrentLog = new DoubleLogEntry(log, "Wrist Motor Current");
        m_wristVoltageLog = new DoubleLogEntry(log, "Wrist Motor Voltage (volts)");
        m_wristTempLog = new DoubleLogEntry(log, "Wrist Motor Temp (deg C)");

    }

    /**
     * Sets the wrist speed.
     *
     * @param speed The value passed in that sets the speed.
     */
    public void moveWrist(double speed) {

        m_motorWrist.set(speed);
        
    }

    public void periodic() {

        

    }
}
