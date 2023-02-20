package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

/**
 * The subsystem responsible for the manipulator.
 */
public class ManipulatorSubsystem extends SubsystemBase {

    private final CANSparkMax m_motorManipulator;
    // private final RelativeEncoder m_manipulatorEncoder;

    private final DoubleLogEntry m_manipulatorMotorOutputLog;
    private final IntegerLogEntry m_manipulatorMotorFaultsLog;
    // private final DoubleLogEntry m_manipulatorMotorPositionLog;
    // private final DoubleLogEntry m_manipulatorMotorVelocityLog;
    private final DoubleLogEntry m_manipulatorMotorCurrentLog;
    private final DoubleLogEntry m_manipulatorMotorVoltageLog;
    private final DoubleLogEntry m_manipulatorMotorTempLog;


    /**
     * Configures hardware inside the manipulator.
     */
    public ManipulatorSubsystem() {

        m_motorManipulator = new CANSparkMax(ManipulatorConstants.MANIPULATOR_MOTOR_CAN_ID, MotorType.kBrushed);

        m_motorManipulator.restoreFactoryDefaults();
        m_motorManipulator.setIdleMode(IdleMode.kBrake);
        m_motorManipulator.setInverted(false);
        m_motorManipulator.setSmartCurrentLimit(40, 40);
        // m_manipulatorEncoder = m_motorManipulator.getEncoder();
        // m_manipulatorEncoder.setPositionConversionFactor(1.0);

        m_motorManipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_motorManipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_motorManipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_motorManipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        DataLog log = DataLogManager.getLog();

        m_manipulatorMotorOutputLog = new DoubleLogEntry(log, "/manipulator/motor/output");
        m_manipulatorMotorFaultsLog = new IntegerLogEntry(log, "/manipulator/motor/faults");
        // m_manipulatorMotorPositionLog = new DoubleLogEntry(log, "/manipulator/motor/position");
        // m_manipulatorMotorVelocityLog = new DoubleLogEntry(log, "/manipulator/motor/velocity");
        m_manipulatorMotorCurrentLog = new DoubleLogEntry(log, "/manipulator/motor/current");
        m_manipulatorMotorVoltageLog = new DoubleLogEntry(log, "/manipulator/motor/voltage");
        m_manipulatorMotorTempLog = new DoubleLogEntry(log, "/manipulator/motor/temp");

    }

    /**
     * Manipulator motor speed controller.
     *
     * @param speed value passed in that controls the motor speed. 
     */
    public void setSpeed(double speed) {

        m_motorManipulator.set(speed);
    }

    

    @Override
    public void periodic() {
        m_manipulatorMotorOutputLog.append(m_motorManipulator.getAppliedOutput());
        m_manipulatorMotorFaultsLog.append(m_motorManipulator.getFaults());
        // m_manipulatorMotorPositionLog.append(m_manipulatorEncoder.getPosition());
        // m_manipulatorMotorVelocityLog.append(m_manipulatorEncoder.getVelocity());
        m_manipulatorMotorCurrentLog.append(m_motorManipulator.getOutputCurrent());
        m_manipulatorMotorVoltageLog.append(m_motorManipulator.getBusVoltage());
        m_manipulatorMotorTempLog.append(m_motorManipulator.getMotorTemperature());

        // SmartDashboard.putNumber("Manipulator Position", m_manipulatorEncoder.getPosition());
        m_motorManipulator.getFaults();

    }

}
