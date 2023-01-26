// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The Class that contains all the subsystems, driver/operator control
 * bindings, and Autonomous commands.
 */
public class RobotContainer {

    private XboxController m_driver;

    private DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    SendableChooser<Command> m_chooser;

    /**
     * The constructor for the robot container.
     */
    public RobotContainer() {

        m_drivetrainSubsystem.setDefaultCommand(new Drive(m_driver, m_drivetrainSubsystem));

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
