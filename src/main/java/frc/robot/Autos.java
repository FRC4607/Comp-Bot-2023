package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Calibrations.ArmCalibrations;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.commands.AutoCollectGamePiece;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.FloorPickup;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.Outtake;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.PlaceGamePiece.PieceLevel;
import frc.robot.commands.Retract;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import java.util.HashMap;
import java.util.List;

/**
 * A wrapper class that used Path Planer lib to generate autos and selects the
 * correct auto for the alliance color.
 */
public class Autos {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final ManipulatorSubsystem m_manipulatorSubsystem;

    private SendableChooser<String> m_chooser;

    private HashMap<String, List<Command>> m_commandMap;

    /**
     * A wrapper class that used Path Planer lib to generate autos and selects the
     * correct auto for the alliance color.
     *
     * @param drivetrainSubsystem  The Drivetrain Subsystem
     * @param elevatorSubsystem    The Elevator Subsystem
     * @param armSubsystem         The Arm Subsystem
     * @param manipulatorSubsystem The Manipulator Subsystem
     */
    public Autos(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem,
            ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;

        // PathPlannerServer.startServer(5811);

        HashMap<String, Command> autoCommands = new HashMap<String, Command>();

        autoCommands.put("Place Top Node",
                new PlaceGamePiece(PieceLevel.TopCone, m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));
        autoCommands.put("Floor Pickup", new FloorPickup(m_elevatorSubsystem, m_armSubsystem, true));
        autoCommands.put("Floor Pickup Fast", new FloorPickup(m_elevatorSubsystem, m_armSubsystem, false));
        autoCommands.put("Collect Piece", new AutoCollectGamePiece(m_manipulatorSubsystem, true));
        autoCommands.put("Collect Piece No Stop",
                new AutoCollectGamePiece(m_manipulatorSubsystem, false).withTimeout(1));
        autoCommands.put("Retract", new Retract(m_elevatorSubsystem, m_armSubsystem));
        autoCommands.put("Balance", new AutoLevel(0.75, drivetrainSubsystem).andThen(new InstantCommand(() -> {
            m_drivetrainSubsystem.setXMode(true);
        })));

        autoCommands.put("Elevator Retracted",
                new MoveElevatorToPosition(ElevatorCalibrations.POSITION_RETRACTED, m_elevatorSubsystem));
        autoCommands.put("Elevator Floor",
                new MoveElevatorToPosition(ElevatorCalibrations.pieceCollection(), m_elevatorSubsystem));
        autoCommands.put("Elevator Top Node",
                new MoveElevatorToPosition(ElevatorCalibrations.nodePositions()[0], m_elevatorSubsystem));

        autoCommands.put("Arm Retracted",
                new MoveArmToPosition(ArmCalibrations.POSITION_RETRACTED, m_armSubsystem));
        autoCommands.put("Arm Floor",
                new MoveArmToPosition(ArmCalibrations.pieceCollection(), m_armSubsystem));
        autoCommands.put("Arm Top Node",
                new MoveArmToPosition(ArmCalibrations.nodePositions()[0], m_armSubsystem));

        autoCommands.put("Outtake", new Outtake(manipulatorSubsystem).withTimeout(0.5));

        // autoCommands.put("Balance", new AutoLevel(0.5, m_drivetrainSubsystem));

        m_chooser = new SendableChooser<>();
        m_commandMap = new HashMap<>();

        PathConstraints normalConstraints = new PathConstraints(Calibrations.SwerveCalibrations.MAX_SPEED_METER * 0.5,
                Calibrations.SwerveCalibrations.MAX_ACCELERATION * 0.5);
        PathConstraints cableCoverConstraints = new PathConstraints(1.0, 1.0);

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_drivetrainSubsystem::getPose,
                m_drivetrainSubsystem::setPose,
                new PIDConstants(3.0, 0, 0),
                new PIDConstants(3.0, 0, 0),
                m_drivetrainSubsystem::setChassisSpeeds,
                autoCommands,
                true,
                m_drivetrainSubsystem);

        m_chooser.addOption("Middle Auto", "Middle Auto");
        m_commandMap.put("Middle Auto", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Middle Auto-Blue", 1.0, 1.5)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Middle Auto-Red", 1.0, 1.5))));

        m_chooser.addOption("Middle Auto No Piece", "Middle Auto No Piece");
        m_commandMap.put("Middle Auto No Piece", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Middle Auto No Piece-Blue", 1.0, 1.5)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Middle Auto No Piece-Red", 1.0, 1.5))));

        PathConstraints movingConstraints = new PathConstraints(3.0, 3.0);
        PathConstraints approachConstraints = new PathConstraints(3.0, 2.5);

        m_chooser.addOption("Two Piece Substation & Dock", "Two Piece Substation & Dock");
        m_commandMap.put("Two Piece Substation & Dock", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Two Piece Substation & Dock-Blue", movingConstraints,
                        approachConstraints, movingConstraints, movingConstraints)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Two Piece Substation & Dock-Red", movingConstraints,
                        approachConstraints, movingConstraints, movingConstraints))));

        m_chooser.addOption("Two Piece Substation", "Two Piece Substation");
        m_commandMap.put("Two Piece Substation", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Two Piece Substation-Blue", movingConstraints,
                        approachConstraints, movingConstraints, movingConstraints, approachConstraints)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Two Piece Substation-Red", movingConstraints,
                        approachConstraints, movingConstraints, movingConstraints, approachConstraints))));

        m_chooser.addOption("One Piece Wall", "One Piece Wall");
        m_commandMap.put("One Piece Wall", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("One Piece Wall-Blue", 1.0, 1.0)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("One Piece Wall-Red", 1.0, 1.0))));

        m_chooser.addOption("Two Piece Wall", "Two Piece Wall");
        m_commandMap.put("Two Piece Wall", List.of(
                autoBuilder.fullAuto(
                        PathPlanner.loadPathGroup("Two Piece Wall-Blue", normalConstraints, cableCoverConstraints,
                                normalConstraints, normalConstraints, cableCoverConstraints, normalConstraints)),
                autoBuilder.fullAuto(
                        PathPlanner.loadPathGroup("Two Piece Wall-Red", normalConstraints, cableCoverConstraints,
                                cableCoverConstraints, normalConstraints, cableCoverConstraints, normalConstraints))));

        m_chooser.setDefaultOption("Place One Piece", "Place One Piece");
        m_commandMap.put("Place One Piece", List.of(
                new PlaceGamePiece(PieceLevel.TopCone, elevatorSubsystem, armSubsystem, manipulatorSubsystem),
                new PlaceGamePiece(PieceLevel.TopCone, elevatorSubsystem, armSubsystem, manipulatorSubsystem)));

        m_chooser.setDefaultOption("Test", "Test");
        m_commandMap.put("Test", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Two Piece Substation-Blue Copy",
                        Calibrations.SwerveCalibrations.MAX_SPEED_METER * 0.75,
                        Calibrations.SwerveCalibrations.MAX_ACCELERATION * 0.75)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("Two Piece Substation-Blue Copy",
                        Calibrations.SwerveCalibrations.MAX_SPEED_METER * 0.75,
                        Calibrations.SwerveCalibrations.MAX_ACCELERATION * 0.75))));

        m_chooser.setDefaultOption("No Auto", "No Auto");
        m_commandMap.put("No Auto", List.of(new InstantCommand(), new InstantCommand()));

        SmartDashboard.putData(m_chooser);
    }

    public Command getAutonomousCommand() {
        String auto = m_chooser.getSelected();
        return m_commandMap.get(auto).get(DriverStation.getAlliance() == Alliance.Blue ? 0 : 1);
    }
}
