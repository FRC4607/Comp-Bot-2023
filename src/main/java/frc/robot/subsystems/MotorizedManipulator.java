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
 * The subsystem responsible for the manipulator.
 */
public class MotorizedManipulator extends SubsystemBase {

    private final CANSparkMax m_motorManipulator;
    private final RelativeEncoder m_manipulatorEncoder;

    private final DoubleLogEntry m_manipulatorPositionLog;
    private final DoubleLogEntry m_manipulatorVelocityLog;
    private final DoubleLogEntry m_manipulatorCurrentLog;
    private final DoubleLogEntry m_manipulatorVoltageLog;
    private final DoubleLogEntry m_manipulatorTempLog;

    /**
     * Configures hardware inside the manipulator.
     */
    public MotorizedManipulator() {

        m_motorManipulator = new CANSparkMax(ManipulatorConstants.MANIPULATOR_MOTOR_CANID, MotorType.kBrushless);

        m_motorManipulator.restoreFactoryDefaults();
        m_motorManipulator.setIdleMode(IdleMode.kBrake);
        m_motorManipulator.setInverted(false);
        m_motorManipulator.setSmartCurrentLimit(20, 40);
        m_manipulatorEncoder = m_motorManipulator.getEncoder();
        m_manipulatorEncoder.setPositionConversionFactor(0);

        m_motorManipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_motorManipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_motorManipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_motorManipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        DataLog log = DataLogManager.getLog();

        m_manipulatorPositionLog = new DoubleLogEntry(log, "Left Motor Position (deg)");
        m_manipulatorVelocityLog = new DoubleLogEntry(log, "Left Motor Velocity (deg)");
        m_manipulatorCurrentLog = new DoubleLogEntry(log, "Left Motor Current");
        m_manipulatorVoltageLog = new DoubleLogEntry(log, "Left Motor Voltage (volts)");
        m_manipulatorTempLog = new DoubleLogEntry(log, "Left Motor Temp (deg C)");
    }

    /**
     * Manipulator motor speed controller.
     *
     * @param speed value passed in that controls the motor speed. 
     */
    public void moveManipulator(double speed) {

        m_motorManipulator.set(speed);

    }

    public void periodic() {



    }

}
