// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveManipulator;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MotorizedManipulator;
import frc.robot.subsystems.WristSubsystem;

/**
 * The Class that contains all the subsystems, driver/operator control
 * bindings, and Autonomous commands.
 */
public class RobotContainer {

    private XboxController m_driver;

    private DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

    private WristSubsystem m_wristSubsystem = new WristSubsystem();

    private MotorizedManipulator m_motorizedManipulator;

    SendableChooser<Command> m_chooser;

    /**
     * The constructor for the robot container.
     */
    public RobotContainer() {

        m_drivetrainSubsystem.setDefaultCommand(new Drive(m_driver, m_drivetrainSubsystem));

        m_elevatorSubsystem.setDefaultCommand(new MoveElevator(m_driver, m_elevatorSubsystem));

        m_wristSubsystem.setDefaultCommand(new MoveWrist(() -> {return m_driver.getRawButton(0); }, () -> {return m_driver.getRawButton(1); }, m_wristSubsystem));

        m_motorizedManipulator.setDefaultCommand(new MoveManipulator(() -> {return m_driver.getRawButton(2); }, () -> {return m_driver.getRawButton(3); }, m_motorizedManipulator));

        configureBindings();

        
        m_chooser = new SendableChooser<>();

        // Put Auto commands here.

        SmartDashboard.putData("Autonomous Command", m_chooser);
    }

    private void configureBindings() {
    }

    /**
     * A method for getting the selected Auto from the Driver's Station.
     *
     * @return The command selected by the drivers
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
