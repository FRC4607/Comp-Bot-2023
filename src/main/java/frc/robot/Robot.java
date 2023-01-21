// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.ADIS16470;

/**
 * The Robot Class that contains all the code.
 */
public class Robot extends TimedRobot {
    private DoubleLogEntry m_gyroAnglLog;
    private DoubleLogEntry m_gyroTempLog;

    private ADIS16470 m_gyro = new ADIS16470();

    // private Command m_autonomousCommand;

    // private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        // m_robotContainer = new RobotContainer();
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        m_gyroAnglLog = new DoubleLogEntry(log, "Gryo Angle (deg)");
        m_gyroTempLog = new DoubleLogEntry(log, "Gryo Temp (deg C)");
        m_gyroAnglLog.append(m_gyro.getAngle());
        m_gyroTempLog.append(m_gyro.getTemp());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        double gyroAngle = m_gyro.getAngle();
        double gyroTemp = m_gyro.getTemp();
        m_gyroAnglLog.append(gyroAngle);
        m_gyroTempLog.append(gyroTemp);
        SmartDashboard.putNumber("Gyro Angle", gyroAngle);
        SmartDashboard.putNumber("Gyro Temp", gyroTemp);
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.schedule();
        // }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.cancel();
        // }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        // CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
