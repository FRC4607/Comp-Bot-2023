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

        // Kind of tuned - So little error because of feed forward that it is hard to tune.
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        public static final double DRIVE_FF_KS = 0.16;
        public static final double DRIVE_FF_KV = 2.6825;
        public static final double DRIVE_FF_KA = 0.808625;

        /**
         * Max acceleration in Meters / s. Experimentally determined at voltage of 9
         * volts.
         */
        public static final double MAX_SPEED_METER = 3.27;
        /**
         * Max acceleration in Meters / s^2. Experimentally determined with a step
         * voltage of 9 volts.
         */
        public static final double MAX_ACCELERATION = 3.27;

    }

    /**
     * Calibrations for the arm subsystems.
     *
     * <p> Units of Position is Deg
     */
    public static final class ArmCalibrations {

        public static final double KS = 0.321;
        public static final double KV = 0.046;
        public static final double KA = 0.0;
        public static final double KG = 1.5;

        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double MIN_POSITION = 20.0;
        public static final double MAX_POSITION = 170.0;
        public static final double MAX_VELOCITY = 295.0;
        public static final double MAX_ACCELERATION = 1920.0 / 5.0 / 2;

        public static final double ELEVATOR_CLEARANCE = 30.0;

        public static final double TOLERANCE = 5.0;

        public static final double ARM_OPERATOR_SPEED = 2.0;

        public static final double POSITION_RETRACTED = 20.0;
        public static final double SHELF_PICKUP_STATIC = 40.0;
        public static final double PIECE_COLLECTION_STATIC = 170.0;
        public static final double[] NODE_POSITIONS_STATIC = { 40, 70, 45, 70 };


        /**
         * Initializes the preset arm positions into the preferences tables if the value is not initialized.
         */
        public static void initPreferences() {
            Preferences.initDouble("ArmPieceCollection", PIECE_COLLECTION_STATIC);
            Preferences.initDouble("ArmShelfPickup", SHELF_PICKUP_STATIC);

            Preferences.initDouble("ArmTopCone", NODE_POSITIONS_STATIC[0]);
            Preferences.initDouble("ArmTopCube", NODE_POSITIONS_STATIC[1]);
            Preferences.initDouble("ArmMiddleCone", NODE_POSITIONS_STATIC[2]);
            Preferences.initDouble("ArmMiddleCube", NODE_POSITIONS_STATIC[3]);
        }

        public static double pieceCollection() {
            return Preferences.getDouble("ArmPieceCollection", PIECE_COLLECTION_STATIC);
        }

        public static double shelfPickup() {
            return Preferences.getDouble("ArmShelfPickup", SHELF_PICKUP_STATIC);
        }

        /**
         * Gets the setpoint from Preferences table.
         *
         * @return The setpoint form the preferences table.
         */
        public static double[] nodePositions() {
            double[] preferences = {
                Preferences.getDouble("ArmTopCone", NODE_POSITIONS_STATIC[0]),
                Preferences.getDouble("ArmTopCube", NODE_POSITIONS_STATIC[1]),
                Preferences.getDouble("ArmMiddleCone", NODE_POSITIONS_STATIC[2]),
                Preferences.getDouble("ArmMiddleCube", NODE_POSITIONS_STATIC[3])
            };
    
            return preferences;
        }    
    }

    /**
     * Calibrations for the Elevator Subsystem.
     * 
     * <p> Units of Position is Rot of the drum
     */
    public static final class ElevatorCalibrations {

        public static final double KS = 0.3375;
        public static final double KV = 1.95;
        public static final double KA = 0.0;
        public static final double KG = 0.25;

        public static final double KP = 10.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double MAX_POSITION = 5.5;
        public static final double MAX_VELOCITY = 4.0;
        public static final double MAX_ACCELERATION = 34.0;
        
        public static final double ARM_CLEARANCE = 35.0;
        
        public static final double TOLERANCE = 0.05;

        public static final double ELEVATOR_DRIVER_SPEED = 4.0 / 50.0;

        private static final double PIECE_COLLECTION_STATIC = 2.475;
        private static final double SHELF_PICKUP_STATIC = 3.3;
        public static final double[] NODE_POSITIONS_STATIC = { 5.08, 5.08, 3.0, 3.25 };

        /**
         * Initializes the preset arm positions into the preferences tables if the value is not initialized.
         */
        public static void initPreferences() {
            Preferences.initDouble("ElevatorPieceCollection", PIECE_COLLECTION_STATIC);
            Preferences.initDouble("ElevatorShelfPickup", SHELF_PICKUP_STATIC);

            Preferences.initDouble("ElevatorTopCone", NODE_POSITIONS_STATIC[0]);
            Preferences.initDouble("ElevatorTopCube", NODE_POSITIONS_STATIC[1]);
            Preferences.initDouble("ElevatorMiddleCone", NODE_POSITIONS_STATIC[2]);
            Preferences.initDouble("ElevatorMiddleCube", NODE_POSITIONS_STATIC[3]);
        }

        public static double pieceCollection() {
            return Preferences.getDouble("ElevatorPieceCollection", PIECE_COLLECTION_STATIC);
        }

        public static double shelfPickup() {
            return Preferences.getDouble("ElevatorShelfPickup", SHELF_PICKUP_STATIC);
        }

        /**
         * Gets the setpoint from Preferences table.
         *
         * @return The setpoint form the preferences table.
         */
        public static double[] nodePositions() {
            double[] preferences = {
                Preferences.getDouble("ElevatorTopCone", NODE_POSITIONS_STATIC[0]),
                Preferences.getDouble("ElevatorTopCube", NODE_POSITIONS_STATIC[1]),
                Preferences.getDouble("ElevatorMiddleCone", NODE_POSITIONS_STATIC[2]),
                Preferences.getDouble("ElevatorMiddleCube", NODE_POSITIONS_STATIC[3])
            };
    
            return preferences;
        }
    }

    /**
     * The Calibrations for the Manipulator Subsystem.
     */
    public static final class ManipulatorCalibrations {
        public static final double INTAKE_SPEED = -0.8;
        public static final double OUTTAKE_SPEED = 0.5;
        public static final double HOLD_SPEED = -0.1;

    }

}
