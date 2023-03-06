package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

/**
 * The subsystem responsible for the manipulator.
 */
public class ManipulatorSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor;
    private final RelativeEncoder m_motorEncoder;

    private final DoubleLogEntry m_motorOutputLog;
    private final IntegerLogEntry m_motorFaultsLog;
    private final DoubleLogEntry m_motorPositionLog;
    private final DoubleLogEntry m_motorVelocityLog;
    private final DoubleLogEntry m_motorCurrentLog;
    private final DoubleLogEntry m_motorVoltageLog;
    private final DoubleLogEntry m_motorTempLog;
    private final DoubleLogEntry m_filteredMotorCurrentLog;

    private final LinearFilter m_filter;
    private double m_filteredCurrent;

    /**
     * Configures hardware inside the manipulator.
     */
    public ManipulatorSubsystem() {

        m_motor = new CANSparkMax(ManipulatorConstants.MANIPULATOR_MOTOR_CAN_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(20, 20);
        m_motorEncoder = m_motor.getEncoder();
        m_motorEncoder.setPositionConversionFactor(1.0);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Max Period - Analog Sensor
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Max Period - Alternate Encoder
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); // Max Period - Duty Cycle Encoder Position
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); // Max Period - Duty Cycle Encoder Velocity

        DataLog log = DataLogManager.getLog();

        m_motorOutputLog = new DoubleLogEntry(log, "/manipulator/motor/output");
        m_motorFaultsLog = new IntegerLogEntry(log, "/manipulator/motor/faults");
        m_motorPositionLog = new DoubleLogEntry(log, "/manipulator/motor/position");
        m_motorVelocityLog = new DoubleLogEntry(log, "/manipulator/motor/velocity");
        m_motorCurrentLog = new DoubleLogEntry(log, "/manipulator/motor/current");
        m_motorVoltageLog = new DoubleLogEntry(log, "/manipulator/motor/voltage");
        m_motorTempLog = new DoubleLogEntry(log, "/manipulator/motor/temp");

        m_filteredMotorCurrentLog = new DoubleLogEntry(log, "/manipulator/motor/currentFiltered");

        m_filter = LinearFilter.movingAverage(25);

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
        return m_filteredCurrent;
    }

    public double getSpeed() {
        return m_motorEncoder.getVelocity();
    }

    

    @Override
    public void periodic() {

        m_filteredCurrent = m_filter.calculate(m_motor.getOutputCurrent());

        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);

        m_motorOutputLog.append(m_motor.getAppliedOutput(), timeStamp);
        m_motorFaultsLog.append(m_motor.getFaults(), timeStamp);
        m_motorPositionLog.append(m_motorEncoder.getPosition(), timeStamp);
        m_motorVelocityLog.append(m_motorEncoder.getVelocity(), timeStamp);
        m_motorCurrentLog.append(m_motor.getOutputCurrent(), timeStamp);
        m_motorVoltageLog.append(m_motor.getBusVoltage(), timeStamp);
        m_motorTempLog.append(m_motor.getMotorTemperature(), timeStamp);

        m_filteredMotorCurrentLog.append(m_filteredCurrent, timeStamp);
    }

}
