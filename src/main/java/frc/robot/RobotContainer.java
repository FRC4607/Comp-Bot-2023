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
import frc.robot.commands.AutoCollectGamePiece;
import frc.robot.commands.CalibrateDriveFF;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveWithSmartDashboard;
import frc.robot.commands.FloorPickup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmSmartDashboard;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorSmartDashboard;
import frc.robot.commands.MoveManipulator;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.PlaceGamePiece.PieceLevel;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.Retract;
import frc.robot.commands.ShelfPickup;
import frc.robot.commands.SwerveSetHomes;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PDHSubsystem;
import java.util.HashMap;

/**
 * The Class that contains all the subsystems, driver/operator control
 * bindings, and Autonomous commands.
 */
public class RobotContainer {

    private static RobotContainer m_instance;

    /**
     * Gets the instance of the robot container.
     *
     * @return The robot Container
     */
    public static RobotContainer getInstance() {
        if (m_instance == null) {
            m_instance = new RobotContainer();
        }
        return m_instance;
    }

    private XboxController m_driver;
    private XboxController m_operator;

    public DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    // public ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    // public ArmSubsystem m_armSubsystem = new ArmSubsystem();
    // public ManipulatorSubsystem m_motorizedManipulator = new ManipulatorSubsystem();
    public PDHSubsystem m_pdhSubsystem = new PDHSubsystem();

    private SendableChooser<Command> m_chooser;

    /**
     * The constructor for the robot container.
     */
    private RobotContainer() {

        m_driver = new XboxController(0);
        m_operator = new XboxController(1);

        m_drivetrainSubsystem.setDefaultCommand(new Drive(m_driver, m_drivetrainSubsystem));
        // m_elevatorSubsystem.setDefaultCommand(new MoveElevator(m_driver, m_elevatorSubsystem));

        // m_motorizedManipulator.setDefaultCommand(
        //         new MoveManipulator(m_operator::getLeftBumper, m_operator::getRightBumper, m_motorizedManipulator));
        // m_armSubsystem.setDefaultCommand(new MoveArm(m_operator, m_armSubsystem));

        configureBindings();

        SmartDashboard.putData(new SwerveSetHomes(m_drivetrainSubsystem));
        // SmartDashboard.putData(new CalibrateTurnFF(m_drivetrainSubsystem));
        SmartDashboard.putData(new CalibrateDriveFF(m_drivetrainSubsystem));
        // SmartDashboard.putData(new ControlSwerveModule(m_drivetrainSubsystem));
        // SmartDashboard.putData(new MoveArmSmartDashboard(m_armSubsystem));
        // SmartDashboard.putData(new MoveElevatorSmartDashboard(m_elevatorSubsystem));
        // SmartDashboard.putData(
        // new PlaceGamePiece(PieceLevel.MiddleCone, m_elevatorSubsystem, m_armSubsystem, m_motorizedManipulator));
        // SmartDashboard.putData(new AutoCollectGamePiece(m_elevatorSubsystem, m_armSubsystem, m_motorizedManipulator));

        SmartDashboard.putData(new DriveWithSmartDashboard(m_drivetrainSubsystem));

        m_chooser = new SendableChooser<>();

        PathPlannerServer.startServer(5811);

        HashMap<String, Command> autoCommands = new HashMap<String, Command>();

        // autoCommands.put("Place Top Cone",
        //         new PlaceGamePiece(PieceLevel.TopCone, m_elevatorSubsystem, m_armSubsystem, m_motorizedManipulator));
        // autoCommands.put("Collect Cone",
        //         new AutoCollectGamePiece(m_elevatorSubsystem, m_armSubsystem, m_motorizedManipulator));

        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //         m_drivetrainSubsystem::getPose,
        //         m_drivetrainSubsystem::setPose,
        //         new PIDConstants(3.0, 0, 0),
        //         new PIDConstants(1.0, 0, 0),
        //         m_drivetrainSubsystem::setChassisSpeeds,
        //         autoCommands,
        //         true,
        //         m_drivetrainSubsystem);

        // m_chooser.setDefaultOption("One Cone",
        //         autoBuilder.fullAuto(PathPlanner.loadPath("One Cone", new PathConstraints(1.0, 1.0))));
        // m_chooser.addOption("Two Cone Substation",
        //         autoBuilder.fullAuto(PathPlanner.loadPath("Two Cone Substation", new PathConstraints(1.5, 1.5))));
        // m_chooser.addOption("Two Cone Wall",
        //         autoBuilder.fullAuto(PathPlanner.loadPath("Two Cone Wall", new PathConstraints(1.5, 1.5))));

        // SmartDashboard.putData("Autonomous Command", m_chooser);
    }

    private void configureBindings() {
        JoystickButton driverStart = new JoystickButton(m_driver, XboxController.Button.kStart.value);
        driverStart.onTrue(new ResetHeading(m_drivetrainSubsystem));

        JoystickButton driverLeftBumper = new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value);
        // driverLeftBumper.onTrue(new FloorPickup(m_elevatorSubsystem, m_armSubsystem));

        JoystickButton driverRightBumper = new JoystickButton(m_driver, XboxController.Button.kRightBumper.value);
        // driverRightBumper.onTrue(new ShelfPickup(m_elevatorSubsystem, m_armSubsystem));

        JoystickButton driverA = new JoystickButton(m_driver, XboxController.Button.kA.value);
        JoystickButton driverB = new JoystickButton(m_driver, XboxController.Button.kB.value);
        // driverB.onTrue(new Retract(m_elevatorSubsystem, m_armSubsystem));

        JoystickButton driverY = new JoystickButton(m_driver, XboxController.Button.kY.value);
        JoystickButton driverX = new JoystickButton(m_driver, XboxController.Button.kX.value);

        JoystickButton operatorA = new JoystickButton(m_operator, XboxController.Button.kA.value);
        JoystickButton operatorB = new JoystickButton(m_operator, XboxController.Button.kB.value);
        // operatorB.onTrue(
                // new PlaceGamePiece(PieceLevel.MiddleCone, m_elevatorSubsystem, m_armSubsystem, m_motorizedManipulator));

        JoystickButton operatorY = new JoystickButton(m_operator, XboxController.Button.kY.value);
        // operatorY.onTrue(
                // new PlaceGamePiece(PieceLevel.TopCone, m_elevatorSubsystem, m_armSubsystem, m_motorizedManipulator));
                
        JoystickButton operatorX = new JoystickButton(m_operator, XboxController.Button.kX.value);



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
