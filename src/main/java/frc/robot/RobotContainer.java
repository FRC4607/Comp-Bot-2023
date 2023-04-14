// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.commands.AutoCollectGamePiece;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.CalibrateDriveFF;
import frc.robot.commands.Drive;
import frc.robot.commands.FloorPickup;
import frc.robot.commands.Intake;
import frc.robot.commands.LLAlignment;
import frc.robot.commands.MoveArmSmartDashboard;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorSmartDashboard;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.OperatorIndicatorLights;
import frc.robot.commands.Outtake;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.Retract;
import frc.robot.commands.ShelfPickup;
import frc.robot.commands.SwerveSetHomes;
import frc.robot.commands.UprightConePickup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndicatorSubsystem;
import frc.robot.subsystems.IndicatorSubsystem.PieceIndicatorState;
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

        RobotController.setBrownoutVoltage(6.3);

        m_driver = new XboxController(0);
        m_operator = new XboxController(1);

        m_drivetrainSubsystem.setDefaultCommand(new Drive(m_driver, m_drivetrainSubsystem));
        m_elevatorSubsystem.setDefaultCommand(new MoveElevator(m_driver, m_elevatorSubsystem));

        m_indicatorSubsystem.setDefaultCommand(new OperatorIndicatorLights(m_operator, m_indicatorSubsystem));

        // m_armSubsystem.setDefaultCommand(new MoveArm(m_operator, m_armSubsystem));

        configureBindings();

        SmartDashboard.putData(new SwerveSetHomes(m_drivetrainSubsystem));

        // SmartDashboard.putData(new CalibrateTurnFF(m_drivetrainSubsystem));
        SmartDashboard.putData(new CalibrateDriveFF(m_drivetrainSubsystem));
        // SmartDashboard.putData(new CalibrateElevatorFF(m_elevatorSubsystem));
        // SmartDashboard.putData(new CalibrateArmFF(m_armSubsystem));
        // SmartDashboard.putData(new ControlSwerveModule(m_drivetrainSubsystem));
        SmartDashboard.putData(new MoveElevatorSmartDashboard(m_elevatorSubsystem));
        SmartDashboard.putData(new MoveArmSmartDashboard(m_armSubsystem));
        // SmartDashboard.putData(
        // new PlaceGamePiece(PieceLevel.MiddleCone, m_elevatorSubsystem,
        // m_armSubsystem, m_motorizedManipulator));

        // SmartDashboard.putData(new DriveWithSmartDashboard(m_drivetrainSubsystem));

        m_autos = new Autos(m_drivetrainSubsystem, m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    }

    private void configureBindings() {
        JoystickButton driverStart = new JoystickButton(m_driver, XboxController.Button.kStart.value);
        driverStart.onTrue(new ResetHeading(m_drivetrainSubsystem));

        JoystickButton driverBack = new JoystickButton(m_driver, XboxController.Button.kBack.value);
        driverBack.onTrue(new InstantCommand(() -> {
            m_drivetrainSubsystem.toggleXMode();
        }));

        JoystickButton driverLeftBumper = new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value);
        // driverLeftBumper.onTrue(new FloorPickup(m_elevatorSubsystem, m_armSubsystem,
        // false));
        driverLeftBumper.onTrue(new SequentialCommandGroup(
                new FloorPickup(m_elevatorSubsystem, m_armSubsystem, false),
                new AutoCollectGamePiece(m_manipulatorSubsystem, false),
                new Retract(m_elevatorSubsystem, m_armSubsystem)));

        JoystickButton driverRightBumper = new JoystickButton(m_driver, XboxController.Button.kRightBumper.value);
        // driverRightBumper.onTrue(new ShelfPickup(m_elevatorSubsystem,
        // m_armSubsystem));
        driverRightBumper.onTrue(new SequentialCommandGroup(
                new ShelfPickup(m_elevatorSubsystem, m_armSubsystem),
                new Intake(m_manipulatorSubsystem).until(driverRightBumper),
                new Retract(m_elevatorSubsystem, m_armSubsystem)));

        JoystickButton driverA = new JoystickButton(m_driver, XboxController.Button.kA.value);
        driverA.onTrue(new UprightConePickup(m_elevatorSubsystem, m_armSubsystem));
        // driverA.onTrue(new SequentialCommandGroup(
        //         new UprightConePickup(m_elevatorSubsystem, m_armSubsystem),
        //         new AutoCollectGamePiece(m_manipulatorSubsystem, false),
        //         new Retract(m_elevatorSubsystem, m_armSubsystem)));

        JoystickButton driverB = new JoystickButton(m_driver, XboxController.Button.kB.value);
        driverB.onTrue(new Retract(m_elevatorSubsystem, m_armSubsystem));

        JoystickButton driverY = new JoystickButton(m_driver, XboxController.Button.kY.value);
        // driverY.whileTrue(new LLAlignment(0, m_driver, m_drivetrainSubsystem));
        JoystickButton driverX = new JoystickButton(m_driver, XboxController.Button.kX.value);
        driverX.whileTrue(new LLAlignment(2, m_driver, m_drivetrainSubsystem));

        JoystickButton operatorA = new JoystickButton(m_operator, XboxController.Button.kA.value);
        JoystickButton operatorB = new JoystickButton(m_operator, XboxController.Button.kB.value);
        // operatorB.onTrue(
        // new PlaceGamePiece(PieceLevel.MiddleCone, m_elevatorSubsystem,
        // m_armSubsystem, m_manipulatorSubsystem));

        operatorB.onTrue(new SequentialCommandGroup(
                new MoveArmToPosition(ArmCalibrations.POSITION_RETRACTED, m_armSubsystem),
                new MoveElevatorToPosition(ElevatorCalibrations::middleNode, m_elevatorSubsystem),
                new MoveArmToPosition(ArmCalibrations::middleNode, m_armSubsystem),
                new WaitUntilCommand(operatorB),
                new Outtake(m_manipulatorSubsystem).withTimeout(0.5),
                new Retract(m_elevatorSubsystem, m_armSubsystem)));

        JoystickButton operatorY = new JoystickButton(m_operator, XboxController.Button.kY.value);
        // operatorY.onTrue(
        // new PlaceGamePiece(PieceLevel.TopCone, m_elevatorSubsystem, m_armSubsystem,
        // m_manipulatorSubsystem));

        operatorY.onTrue(new SequentialCommandGroup(
                new MoveArmToPosition(ArmCalibrations.POSITION_RETRACTED, m_armSubsystem),
                new MoveElevatorToPosition(ElevatorCalibrations::topNode, m_elevatorSubsystem),
                new MoveArmToPosition(ArmCalibrations::topNode, m_armSubsystem),
                new WaitUntilCommand(operatorY),
                new Outtake(m_manipulatorSubsystem).withTimeout(0.5),
                new Retract(m_elevatorSubsystem, m_armSubsystem)));

        JoystickButton operatorX = new JoystickButton(m_operator, XboxController.Button.kX.value);
        JoystickButton operatorLeftBumper = new JoystickButton(m_operator, XboxController.Button.kLeftBumper.value);
        operatorLeftBumper.whileTrue(new Intake(m_manipulatorSubsystem));

        JoystickButton operatorRightBumper = new JoystickButton(m_operator, XboxController.Button.kRightBumper.value);
        operatorRightBumper.whileTrue(new Outtake(m_manipulatorSubsystem));

        POVButton operatorDPadUp = new POVButton(m_operator, 0);
        POVButton operatorDPadDown = new POVButton(m_operator, 180);
        operatorDPadUp.onTrue(new InstantCommand(() -> {
            m_indicatorSubsystem.setIndicator(PieceIndicatorState.CONE);
        }));
        operatorDPadUp.onFalse(new InstantCommand(() -> {
            m_indicatorSubsystem.setIndicator(PieceIndicatorState.NONE);
        }));
        operatorDPadDown.onTrue(new InstantCommand(() -> {
            m_indicatorSubsystem.setIndicator(PieceIndicatorState.CUBE);
        }));
        operatorDPadDown.onFalse(new InstantCommand(() -> {
            m_indicatorSubsystem.setIndicator(PieceIndicatorState.NONE);
        }));

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
