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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

/**
 * The subsystem responsible for the manipulator.
 */
public class ManipulatorSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor;
    private final RelativeEncoder m_manipulatorEncoder;

    private final DoubleLogEntry m_manipulatorMotorOutputLog;
    private final IntegerLogEntry m_manipulatorMotorFaultsLog;
    private final DoubleLogEntry m_manipulatorMotorPositionLog;
    private final DoubleLogEntry m_manipulatorMotorVelocityLog;
    private final DoubleLogEntry m_manipulatorMotorCurrentLog;
    private final DoubleLogEntry m_manipulatorMotorVoltageLog;
    private final DoubleLogEntry m_manipulatorMotorTempLog;


    /**
     * Configures hardware inside the manipulator.
     */
    public ManipulatorSubsystem() {

        m_motor = new CANSparkMax(ManipulatorConstants.MANIPULATOR_MOTOR_CAN_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(20, 20);
        m_manipulatorEncoder = m_motor.getEncoder();
        m_manipulatorEncoder.setPositionConversionFactor(1.0);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        DataLog log = DataLogManager.getLog();

        m_manipulatorMotorOutputLog = new DoubleLogEntry(log, "/manipulator/motor/output");
        m_manipulatorMotorFaultsLog = new IntegerLogEntry(log, "/manipulator/motor/faults");
        m_manipulatorMotorPositionLog = new DoubleLogEntry(log, "/manipulator/motor/position");
        m_manipulatorMotorVelocityLog = new DoubleLogEntry(log, "/manipulator/motor/velocity");
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

        m_motor.set(speed);
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    public double getSpeed() {
        return m_manipulatorEncoder.getVelocity();
    }

    

    @Override
    public void periodic() {
        m_manipulatorMotorOutputLog.append(m_motor.getAppliedOutput());
        m_manipulatorMotorFaultsLog.append(m_motor.getFaults());
        m_manipulatorMotorPositionLog.append(m_manipulatorEncoder.getPosition());
        m_manipulatorMotorVelocityLog.append(m_manipulatorEncoder.getVelocity());
        m_manipulatorMotorCurrentLog.append(m_motor.getOutputCurrent());
        m_manipulatorMotorVoltageLog.append(m_motor.getBusVoltage());
        m_manipulatorMotorTempLog.append(m_motor.getMotorTemperature());
    }

}
