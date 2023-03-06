// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The Robot Class that contains all the code.
 */
public class Robot extends TimedRobot {

    private enum AutoRecordState {
        BEFORE_START,
        AUTO,
        AUTO_DISABLED,
        TELEOP,
        STOP
    }

    private AutoRecordState m_autoRecordState = AutoRecordState.BEFORE_START;
    private boolean m_startValueSent = false;
    private boolean m_stopValueSent = false;

    private final NetworkTable m_piTable = NetworkTableInstance.getDefault().getTable("PiTable");
    private final NetworkTableEntry m_recording = m_piTable.getEntry("RecordingEnabled");
    private boolean m_startSet = false;

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer = RobotContainer.getInstance();

    private AddressableLED m_leds;
    private AddressableLEDBuffer m_ledBuffer;
    private NetworkTableEntry m_allianceColorEntry;

    private boolean m_previousIsRed;

    @Override
    public void robotInit() {
        m_recording.setBoolean(false);
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        DriverStation.startDataLog(log);

        m_leds = new AddressableLED(0);
        m_leds.setLength(38);
        m_ledBuffer = new AddressableLEDBuffer(38);
        m_leds.setData(m_ledBuffer);
        m_leds.start();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmsInfo = inst.getTable("FMSInfo");
        m_allianceColorEntry = fmsInfo.getEntry("IsRedAlliance");

        boolean isRed = m_allianceColorEntry.getBoolean(false);

        m_previousIsRed = isRed;
        for (int i = 0; i < 38; i++) {
            m_ledBuffer.setHSV(i, isRed ? 0 : 100, 255, 255);
        }
        m_leds.setData(m_ledBuffer);

        Calibrations.ArmCalibrations.initPreferences();
        Calibrations.ElevatorCalibrations.initPreferences();
    }

    @Override
    public void robotPeriodic() {
        if (m_autoRecordState == AutoRecordState.AUTO && !m_startValueSent) {
            System.out.println("Starting automatic recording.");
            m_recording.setBoolean(true);
            m_startValueSent = true;
        } else if (m_autoRecordState == AutoRecordState.STOP && !m_stopValueSent) {
            System.out.println("Stopping automatic recording.");
            m_recording.setBoolean(false);
            m_stopValueSent = true;
        }
        if (m_piTable.containsKey("Start") && !m_startSet) {
            m_piTable.getEntry("Start").setBoolean(true);
            m_startSet = true;
        }
        CommandScheduler.getInstance().run();

        boolean isRed = m_allianceColorEntry.getBoolean(false);

        if (isRed != m_previousIsRed) {

            for (int i = 0; i < 38; i++) {
                m_ledBuffer.setHSV(i, isRed ? 0 : 100, 255, 255);
            }
            m_leds.setData(m_ledBuffer);
        }
    
    }

    @Override
    public void disabledInit() {
        if (m_autoRecordState == AutoRecordState.AUTO) {
            m_autoRecordState = AutoRecordState.AUTO_DISABLED;
        } else if (m_autoRecordState == AutoRecordState.TELEOP) {
            m_autoRecordState = AutoRecordState.STOP;
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        if (m_autoRecordState == AutoRecordState.BEFORE_START) {
            m_autoRecordState = AutoRecordState.AUTO;
        }

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autoRecordState == AutoRecordState.AUTO_DISABLED) {
            m_autoRecordState = AutoRecordState.TELEOP;
        }


        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
