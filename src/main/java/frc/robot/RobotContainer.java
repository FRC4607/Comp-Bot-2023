// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCollectGamePiece;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.CalibrateArmFF;
import frc.robot.commands.CalibrateDriveFF;
import frc.robot.commands.CalibrateElevatorFF;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveWithSmartDashboard;
import frc.robot.commands.FloorPickup;
import frc.robot.commands.LLAlignment;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmSmartDashboard;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorSmartDashboard;
import frc.robot.commands.MoveManipulator;
import frc.robot.commands.OperatorIndicatorLights;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.PlaceGamePiece.PieceLevel;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.Retract;
import frc.robot.commands.ShelfPickup;
import frc.robot.commands.SwerveSetHomes;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndicatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PDHSubsystem;

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
    public ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public ArmSubsystem m_armSubsystem = new ArmSubsystem();
    public ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem();
    public PDHSubsystem m_pdhSubsystem = new PDHSubsystem();
    public IndicatorSubsystem m_indicatorSubsystem = new IndicatorSubsystem();

    private Autos m_autos;

    /**
     * The constructor for the robot container.
     */
    private RobotContainer() {

        m_driver = new XboxController(0);
        m_operator = new XboxController(1);

        m_drivetrainSubsystem.setDefaultCommand(new Drive(m_driver, m_drivetrainSubsystem));
        m_elevatorSubsystem.setDefaultCommand(new MoveElevator(m_driver, m_elevatorSubsystem));

        m_manipulatorSubsystem.setDefaultCommand(
                new MoveManipulator(m_operator::getLeftBumper, m_operator::getRightBumper, m_manipulatorSubsystem));

        m_indicatorSubsystem.setDefaultCommand(new OperatorIndicatorLights(m_operator, m_indicatorSubsystem));
        
        // m_armSubsystem.setDefaultCommand(new MoveArm(m_operator, m_armSubsystem));

        configureBindings();

        SmartDashboard.putData(new SwerveSetHomes(m_drivetrainSubsystem));

        SmartDashboard.putData(new AutoLevel(0.5, m_drivetrainSubsystem));
        // SmartDashboard.putData(new CalibrateTurnFF(m_drivetrainSubsystem));
        // SmartDashboard.putData(new CalibrateDriveFF(m_drivetrainSubsystem));
        // SmartDashboard.putData(new CalibrateElevatorFF(m_elevatorSubsystem));
        // SmartDashboard.putData(new CalibrateArmFF(m_armSubsystem));
        // SmartDashboard.putData(new ControlSwerveModule(m_drivetrainSubsystem));
        // SmartDashboard.putData(new MoveElevatorSmartDashboard(m_elevatorSubsystem));
        // SmartDashboard.putData(new MoveArmSmartDashboard(m_armSubsystem));
        // SmartDashboard.putData(
        // new PlaceGamePiece(PieceLevel.MiddleCone, m_elevatorSubsystem,
        // m_armSubsystem, m_motorizedManipulator));

        // SmartDashboard.putData(new DriveWithSmartDashboard(m_drivetrainSubsystem));

        m_autos = new Autos(m_drivetrainSubsystem, m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    }

    private void configureBindings() {
        JoystickButton driverStart = new JoystickButton(m_driver, XboxController.Button.kStart.value);
        driverStart.onTrue(new ResetHeading(m_drivetrainSubsystem));

        JoystickButton driverLeftBumper = new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value);
        driverLeftBumper.onTrue(new SequentialCommandGroup(
                new FloorPickup(m_elevatorSubsystem, m_armSubsystem),
                new AutoCollectGamePiece(m_manipulatorSubsystem, false),
                new Retract(m_elevatorSubsystem, m_armSubsystem)));

        JoystickButton driverRightBumper = new JoystickButton(m_driver, XboxController.Button.kRightBumper.value);
        driverRightBumper.onTrue(new SequentialCommandGroup(
                new ShelfPickup(m_elevatorSubsystem, m_armSubsystem),
                new AutoCollectGamePiece(m_manipulatorSubsystem, false),
                new Retract(m_elevatorSubsystem, m_armSubsystem)));

        JoystickButton driverA = new JoystickButton(m_driver, XboxController.Button.kA.value);
        JoystickButton driverB = new JoystickButton(m_driver, XboxController.Button.kB.value);
        driverB.onTrue(new Retract(m_elevatorSubsystem, m_armSubsystem));

        JoystickButton driverY = new JoystickButton(m_driver, XboxController.Button.kY.value);
        driverY.whileTrue(new LLAlignment(0, m_driver, m_drivetrainSubsystem));
        JoystickButton driverX = new JoystickButton(m_driver, XboxController.Button.kX.value);
        driverX.whileTrue(new LLAlignment(1, m_driver, m_drivetrainSubsystem));

        JoystickButton operatorA = new JoystickButton(m_operator, XboxController.Button.kA.value);
        JoystickButton operatorB = new JoystickButton(m_operator, XboxController.Button.kB.value);
        operatorB.onTrue(
                new PlaceGamePiece(PieceLevel.MiddleCone, m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));

        JoystickButton operatorY = new JoystickButton(m_operator, XboxController.Button.kY.value);
        operatorY.onTrue(
                new PlaceGamePiece(PieceLevel.TopCone, m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));

        JoystickButton operatorX = new JoystickButton(m_operator, XboxController.Button.kX.value);

        // POVButton operatorDPadUp = new POVButton(m_operator, 0);
        // POVButton operatorDPadDown = new POVButton(m_operator, 180);
        // operatorDPadUp.onTrue(new InstantCommand(() -> {
        //     m_indicatorSubsystem.setIndicator(PieceIndicatorState.CONE);
        // }));
        // operatorDPadUp.onFalse(new InstantCommand(() -> {
        //     m_indicatorSubsystem.setIndicator(PieceIndicatorState.NONE);
        // }));
        // operatorDPadDown.onTrue(new InstantCommand(() -> {
        //     m_indicatorSubsystem.setIndicator(PieceIndicatorState.CUBE);
        // }));
        // operatorDPadDown.onFalse(new InstantCommand(() -> {
        //     m_indicatorSubsystem.setIndicator(PieceIndicatorState.NONE);
        // }));

    }

    /**
     * A method for getting the selected Auto from the Driver's Station.
     *
     * @return The command selected by the drivers
     */
    public Command getAutonomousCommand() {
        return m_autos.getAutonomousCommand();
    }
}
