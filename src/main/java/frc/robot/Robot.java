// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.LimelightHelpers;

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

    private DoubleLogEntry m_roborioCanUtilizationLogEntry;
    private IntegerLogEntry m_roborioCanOffCountLogEntry;
    private IntegerLogEntry m_roborioCanRxErrCountLogEntry;
    private IntegerLogEntry m_roborioCanTxErrCountLogEntry;
    private IntegerLogEntry m_roborioCanTxFullCountLogEntry;
    private CANStatus m_canStatus;

    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
        
        m_recording.setBoolean(false);
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        DriverStation.startDataLog(log);
        
        m_roborioCanUtilizationLogEntry = new DoubleLogEntry(log, "RoboRio CAN Utilization");
        m_roborioCanOffCountLogEntry = new IntegerLogEntry(log, "RoboRio CAN Off Count");
        m_roborioCanRxErrCountLogEntry = new IntegerLogEntry(log, "RoboRio CAN Rx Error Count");
        m_roborioCanTxErrCountLogEntry = new IntegerLogEntry(log, "RoboRio CAN Tx Error Count");
        m_roborioCanTxFullCountLogEntry = new IntegerLogEntry(log, "RoboRio CAN Tx Full Count");

        m_canStatus = RobotController.getCANStatus();
        
        m_roborioCanUtilizationLogEntry.append(m_canStatus.percentBusUtilization);
        m_roborioCanOffCountLogEntry.append(m_canStatus.busOffCount);
        m_roborioCanRxErrCountLogEntry.append(m_canStatus.receiveErrorCount);
        m_roborioCanTxErrCountLogEntry.append(m_canStatus.transmitErrorCount);
        m_roborioCanTxFullCountLogEntry.append(m_canStatus.txFullCount);

        Calibrations.ArmCalibrations.initPreferences();
        Calibrations.ElevatorCalibrations.initPreferences();

        LimelightHelpers.setPipelineIndex("limelight", 2);
        LimelightHelpers.setCameraMode_Driver("limelight");
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

        m_canStatus = RobotController.getCANStatus();

        long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);

        m_roborioCanUtilizationLogEntry.append(m_canStatus.percentBusUtilization, timeStamp);
        m_roborioCanOffCountLogEntry.append(m_canStatus.busOffCount, timeStamp);
        m_roborioCanRxErrCountLogEntry.append(m_canStatus.receiveErrorCount, timeStamp);
        m_roborioCanTxErrCountLogEntry.append(m_canStatus.transmitErrorCount, timeStamp);
        m_roborioCanTxFullCountLogEntry.append(m_canStatus.txFullCount, timeStamp);
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
