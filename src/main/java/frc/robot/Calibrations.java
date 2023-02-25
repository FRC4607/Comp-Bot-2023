package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Used for calibrating the robot.
 */
public class Calibrations {

    /**
     * The calibrations for the swerve drivetrain.
     */
    public static final class SwerveCalibrations {

        public static final double TURN_KP = 1.0;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.0;

        /** Max turning speed in RPM. */
        public static final double TURN_MAX_VELOCITY = 4497.14;
        /** Max turning acceleration in Rad / s^2. */
        public static final double TURN_MAX_ACCELERATION = 1851.23;
        /** Min turning speed in Rad / s. */
        public static final double TURN_MIN_VELOCITY = 0.001;

        // Yet to be tuned
        public static final double DRIVE_KP = 0.0;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /*
         * Rear Left motor data was not included because of noise in the data.
         */
        public static final double DRIVE_FF_KS = (0.21 + 0.142 + 0.221 + 0.153 + 0.236 + 0.153) / 6;
        public static final double DRIVE_FF_KV = (2.61 + 2.66 + 2.56 + 2.59 + 2.56 + 2.61) / 6;
        // Too few data points to calculate. Will revisit if time allows.
        public static final double DRIVE_FF_KA = 0.0;

        /**
         * Max acceleration in Meters / s. Experimentally determined at voltage of 9
         * volts.
         */
        public static final double MAX_SPEED_METER = 3.39;
        /**
         * Max acceleration in Meters / s^2. Experimentally determined with a step
         * voltage of 9 volts.
         */
        public static final double MAX_ACCELERATION = 4.39;

    }

    /**
     * Calibrations for the arm subsystems.
     */
    public static final class ArmCalibrations {

        public static final double POSITION_PIECE_COLLECTION = 170.0;
        public static final double POSITION_LEVEL = 80.0;
        public static final double POSITION_TOP_NODE = 90.0;
        public static final double POSITION_RETRACTED = 10.0;

        public static final double[] ARM_PLACEMENT_POSITIONS_STATIC = { 72, 70, 45, 70 };

        /**
         * Gets the setpoint from Preferences table.
         *
         * @return The setpoint form the preferences table.
         */
        public static double[] armPlacementPositions() {
            double[] preferences = {
                Preferences.getDouble("TopConeArm", 0),
                Preferences.getDouble("TopCubeArm", 0),
                Preferences.getDouble("MiddleConeArm", 0),
                Preferences.getDouble("MiddleCubeArm", 0)
            };

            if (preferences[0] == 0) {
                Preferences.setDouble("TopConeArm", ARM_PLACEMENT_POSITIONS_STATIC[0]);
                preferences[0] = ARM_PLACEMENT_POSITIONS_STATIC[0];
            }

            if (preferences[1] == 0) {
                Preferences.setDouble("TopCubeArm", ARM_PLACEMENT_POSITIONS_STATIC[1]);
                preferences[1] = ARM_PLACEMENT_POSITIONS_STATIC[1];
            }

            if (preferences[2] == 0) {
                Preferences.setDouble("MiddleConeArm", ARM_PLACEMENT_POSITIONS_STATIC[2]);
                preferences[2] = ARM_PLACEMENT_POSITIONS_STATIC[2];
            }

            if (preferences[3] == 0) {
                Preferences.setDouble("MiddleCubeArm", ARM_PLACEMENT_POSITIONS_STATIC[3]);
                preferences[3] = ARM_PLACEMENT_POSITIONS_STATIC[3];
            }

            return preferences;
        }

        public static final double KG = 0.5;

        public static final double ELEVATOR_CLEARANCE = 30.0;


        public static final double KP = 0.15;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double MAX_VELOCITY = 180.0;
        public static final double MAX_ACCELERATION = 120.0;

        public static final double TOLERANCE = 4.0;

        public static final double MIN_POSITION = 10.0;
        public static final double MAX_POSITION = 180.0;

        public static final double ARM_SPEED = 2.0;
    }

    /**
     * Calibrations for the Elevator Subsystem.
     */
    public static final class ElevatorCalibrations {

        public static final double[] ELEVATOR_PLACEMENT_POSITIONS_STATIC = { 92.3, 60, 55, 40 };

        /**
         * Gets the setpoint from Preferences table.
         *
         * @return The setpoint form the preferences table.
         */
        public static double[] elevatorPlacementPositions() {
            double[] preferences = {
                Preferences.getDouble("TopConeElevator", 0),
                Preferences.getDouble("TopCubeElevator", 0),
                Preferences.getDouble("MiddleConeElevator", 0),
                Preferences.getDouble("MiddleCubeElevator", 0)
            };

            if (preferences[0] == 0) {
                Preferences.setDouble("TopConeElevator", ELEVATOR_PLACEMENT_POSITIONS_STATIC[0]);
                preferences[0] = ELEVATOR_PLACEMENT_POSITIONS_STATIC[0];
            }

            if (preferences[1] == 0) {
                Preferences.setDouble("TopCubeElevator", ELEVATOR_PLACEMENT_POSITIONS_STATIC[1]);
                preferences[1] = ELEVATOR_PLACEMENT_POSITIONS_STATIC[1];
            }

            if (preferences[2] == 0) {
                Preferences.setDouble("MiddleConeElevator", ELEVATOR_PLACEMENT_POSITIONS_STATIC[2]);
                preferences[2] = ELEVATOR_PLACEMENT_POSITIONS_STATIC[2];
            }

            if (preferences[3] == 0) {
                Preferences.setDouble("MiddleCubeElevator", ELEVATOR_PLACEMENT_POSITIONS_STATIC[3]);
                preferences[3] = ELEVATOR_PLACEMENT_POSITIONS_STATIC[3];
            }

            return preferences;
        }

        public static final double KP = 1.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KFF = 0.0;

        public static final double KG = 0.2;
        public static final double MIDDLE_ROW_HEIGHT = 75;
        public static final double MAX_ACCELERATION = 100;
        public static final double MAX_VELOCITY = 100;

        public static final double TOLERANCE = 1.5;

        public static final double POSITION_PIECE_COLLECTION = 45.0;
        public static final double ARM_CLEARANCE = 35.0;
        public static final double ELEVATOR_DRIVER_SPEED = 1.0;
    }

    /**
     * The Calibrations for the Manipulator Subsystem.
     */
    public static final class ManipulatorCalibrations {
        public static final double INTAKE_SPEED = -0.75;
        public static final double OUTTAKE_SPEED = 0.2;
        public static final double HOLD_SPEED = -0.1;

    }

}
