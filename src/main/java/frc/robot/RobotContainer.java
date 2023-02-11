// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CalibrateDriveFF;
import frc.robot.commands.CalibrateTurnFF;
import frc.robot.commands.ControlSwerveModule;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveManipulator;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.DriveWithSmartDashboard;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.SwerveSetHomes;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.HashMap;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MotorizedManipulator;
import frc.robot.subsystems.WristSubsystem;

/**
 * The Class that contains all the subsystems, driver/operator control
 * bindings, and Autonomous commands.
 */
public class RobotContainer {

    private XboxController m_driver;

    public DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    public ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public WristSubsystem m_wristSubsystem = new WristSubsystem();
    public MotorizedManipulator m_motorizedManipulator;

    SendableChooser<Command> m_chooser;

    /**
     * The constructor for the robot container.
     */
    public RobotContainer() {

        m_driver = new XboxController(0);
        m_drivetrainSubsystem.setDefaultCommand(new Drive(m_driver, m_drivetrainSubsystem));

        m_elevatorSubsystem.setDefaultCommand(new MoveElevator(m_driver, m_elevatorSubsystem));

        m_wristSubsystem.setDefaultCommand(new MoveWrist(() -> {return m_driver.getRawButton(0); }, () -> {return m_driver.getRawButton(1); }, m_wristSubsystem));

        m_motorizedManipulator.setDefaultCommand(new MoveManipulator(() -> {return m_driver.getRawButton(2); }, () -> {return m_driver.getRawButton(3); }, m_motorizedManipulator));

        configureBindings();

        SmartDashboard.putData(new SwerveSetHomes(m_drivetrainSubsystem));
        // SmartDashboard.putData(new CalibrateTurnFF(m_drivetrainSubsystem));
        // SmartDashboard.putData(new CalibrateDriveFF(m_drivetrainSubsystem));
        SmartDashboard.putData(new ControlSwerveModule(m_drivetrainSubsystem));
        SmartDashboard.putData(new DriveWithSmartDashboard(m_drivetrainSubsystem));

        m_chooser = new SendableChooser<>();

        PathPlannerServer.startServer(5811);

        m_chooser.setDefaultOption("Test Auto", new SwerveAutoBuilder(
                m_drivetrainSubsystem::getPose,
                m_drivetrainSubsystem::setPose,
                new PIDConstants(3.0, 0, 0),
                new PIDConstants(1.0, 0, 0),
                m_drivetrainSubsystem::setChassisSpeeds,
                new HashMap<>(),
                m_drivetrainSubsystem).fullAuto(PathPlanner.loadPath("Test-Auto", new PathConstraints(2.0, 1.5))));

        SmartDashboard.putData("Autonomous Command", m_chooser);
    }

    private void configureBindings() {
        JoystickButton driverStart = new JoystickButton(m_driver, XboxController.Button.kStart.value);

        driverStart.onTrue(new ResetHeading(m_drivetrainSubsystem));
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
