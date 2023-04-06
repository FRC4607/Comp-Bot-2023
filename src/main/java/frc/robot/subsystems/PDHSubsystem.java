package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem used for the elevator lift.
 */
public class PDHSubsystem extends SubsystemBase {

    private final PowerDistribution m_pdh;

    private final DoubleLogEntry m_voltage;
    private final DoubleLogEntry m_temperature;
    private final FloatArrayLogEntry m_currents;

    /**
     * Declares and configures motors.
     */
    public PDHSubsystem() {
   
        m_pdh = new PowerDistribution();

        DataLog log = DataLogManager.getLog();
        m_voltage = new DoubleLogEntry(log, "/pdh/voltage");
        m_temperature = new DoubleLogEntry(log, "/pdh/temperature");
        m_currents = new FloatArrayLogEntry(log, "/pdh/currents");
        
    }

    @Override
    public void periodic() {
        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);


        m_voltage.append(m_pdh.getVoltage(), timeStamp);
        m_temperature.append(m_pdh.getTemperature(), timeStamp);
        float[] currents = new float[m_pdh.getNumChannels()];
        for (int i = 0; i < m_pdh.getNumChannels(); i++) {
            currents[i] = (float) m_pdh.getCurrent(i);
        }
        m_currents.append(currents, timeStamp);
    }
}