package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ManipulatorCalibrations;
import frc.robot.Constants.ManipulatorConstants;

/**
 * The subsystem responsible for the manipulator.
 */
public class ManipulatorSubsystem extends SubsystemBase {

    /**
     * The type of pice the color sensor thinks is in the intake.
     */
    public enum PieceType {
        Cone,
        Cube,
        None
    }

    private final CANSparkMax m_motor;
    private final RelativeEncoder m_motorEncoder;
    // private final ColorSensorV3 m_colorSensor;
    // private final ColorMatch m_colorMatcher;

    private final DoubleLogEntry m_motorOutputLog;
    private final IntegerLogEntry m_motorFaultsLog;
    private final DoubleLogEntry m_motorPositionLog;
    private final DoubleLogEntry m_motorVelocityLog;
    private final DoubleLogEntry m_motorCurrentLog;
    private final DoubleLogEntry m_motorVoltageLog;
    private final DoubleLogEntry m_motorTempLog;
    private final DoubleLogEntry m_filteredMotorCurrentLog;
    // private final StringLogEntry m_colorSensorPieceLog;
    // private final DoubleLogEntry m_colorSensorConfidenceLog;
    // private final DoubleLogEntry m_colorSensorRedLog;
    // private final DoubleLogEntry m_colorSensorGreenLog;
    // private final DoubleLogEntry m_colorSensorBlueLog;
    private final StringLogEntry m_currentCommandLog;

    private final LinearFilter m_filter;
    private double m_filteredCurrent;
    private PieceType m_pieceType;

    /**
     * Configures hardware inside the manipulator.
     */
    public ManipulatorSubsystem() {

        // m_colorSensor =  new ColorSensorV3(I2C.Port.kMXP);
        // m_colorMatcher = new ColorMatch();
        // m_colorMatcher.addColorMatch(ManipulatorCalibrations.CONE);
        // m_colorMatcher.addColorMatch(ManipulatorCalibrations.CUBE);

        // m_colorMatcher.setConfidenceThreshold(ManipulatorCalibrations.MATCHER_CONFIDENCE);

        m_motor = new CANSparkMax(ManipulatorConstants.MANIPULATOR_MOTOR_CAN_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(40, 20);
        m_motorEncoder = m_motor.getEncoder();
        m_motorEncoder.setPositionConversionFactor(1.0);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20); // Faults and Applied Output
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // Velocity, Bus Voltage, Temp, and Current
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40); // Position
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

        // m_colorSensorPieceLog = new StringLogEntry(log, "/manipulator/colorSensor/detectedPiece");
        // m_colorSensorConfidenceLog = new DoubleLogEntry(log, "/manipulator/colorSensor/confidence");
        // m_colorSensorRedLog = new DoubleLogEntry(log, "/manipulator/colorSensor/red");
        // m_colorSensorGreenLog = new DoubleLogEntry(log, "/manipulator/colorSensor/green");
        // m_colorSensorBlueLog = new DoubleLogEntry(log, "/manipulator/colorSensor/blue");

        m_currentCommandLog = new StringLogEntry(log, "/manipulator/command");

        m_filter = LinearFilter.movingAverage(10);

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

        // Color color = m_colorSensor.getColor();

        // ColorMatchResult matchedColor = m_colorMatcher.matchColor(color);

        // if (matchedColor == null) {
        //     m_pieceType = PieceType.None;
        // } else if (matchedColor.color == ManipulatorCalibrations.CONE) {
        //     m_pieceType = PieceType.Cone;
        // } else if (matchedColor.color == ManipulatorCalibrations.CUBE) {
        //     m_pieceType = PieceType.Cube;
        // } else {
        //     m_pieceType = PieceType.None;
        // }

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
        
        // m_colorSensorPieceLog.append(m_pieceType.toString(), timeStamp);
        // m_colorSensorConfidenceLog.append(matchedColor == null ? 0 : matchedColor.confidence, timeStamp);
        // m_colorSensorRedLog.append(color.red, timeStamp);
        // m_colorSensorGreenLog.append(color.green, timeStamp);
        // m_colorSensorBlueLog.append(color.blue, timeStamp);

        Command currentCommand = getCurrentCommand();
        m_currentCommandLog.append(currentCommand != null ? currentCommand.getName() : "None", timeStamp);

        // SmartDashboard.putString("Piece Type", m_pieceType.toString());
        // SmartDashboard.putNumber("Color Sensor Confidence", matchedColor == null ? 0 : matchedColor.confidence);
        // SmartDashboard.putNumber("Color Sensor Red", color.red);
        // SmartDashboard.putNumber("Color Sensor Green", color.green);
        // SmartDashboard.putNumber("Color Sensor Blue", color.blue);
    }

}
