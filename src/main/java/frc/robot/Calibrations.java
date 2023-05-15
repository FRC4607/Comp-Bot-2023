package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.util.Color;

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
        public static final double DRIVE_KP = 0.5;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        public static final double DRIVE_FF_KS = 0.15;
        public static final double DRIVE_FF_KV = 2.415;
        public static final double DRIVE_FF_KA = 0.0;

        /**
         * Max acceleration in Meters / s. Experimentally determined at voltage of 9
         * volts.
         */
        public static final double MAX_SPEED_METER = 4.05;
        /**
         * Max acceleration in Meters / s^2. Experimentally determined with a step
         * voltage of 9 volts.
         */
        public static final double MAX_ACCELERATION = 5.627;
        public static final double MAX_DECELERATION = (6.72 + 12.6) / 2;

    }

    /**
     * Calibrations for the arm subsystems.
     *
     * <p> Units of Position is Deg
     */
    public static final class ArmCalibrations {

        public static final double KS = 0.321;
        public static final double KV = 0.020;
        public static final double KA = 0.0;
        public static final double KG = -0.94;

        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double MIN_POSITION = 20.0;
        public static final double MAX_POSITION = 170.0;
        public static final double MAX_VELOCITY = 295.0 * 0.6;
        public static final double MAX_ACCELERATION = 1920.0 * 0.3;

        public static final double ELEVATOR_CLEARANCE = 30.0;

        public static final double TOLERANCE = 3.0;

        public static final double ARM_OPERATOR_SPEED = 2.0;

        public static final double POSITION_RETRACTED = 20.0;
        public static final double SHELF_PICKUP_STATIC = 30.0;
        public static final double SHELF_PICKUP_ALT_STATIC = 30.0;
        public static final double UPRIGHT_CONE_STATIC = 140.0;
        public static final double PIECE_COLLECTION_STATIC = 165.0;
        public static final double[] NODE_POSITIONS_STATIC = { 40, 70, 45, 70 };
        // This is defined as "what does the encoder read when the system is in the position I want to be zero?"
        // REV tuner read 205.137 when the arm was vertical, subtract 120 to convert to the correct position.
        public static final double ABSOLUTE_ENCODER_OFFSET = MathUtil.inputModulus(144.713 - 120, 0, 360);


        /**
         * Initializes the preset arm positions into the preferences tables if the value is not initialized.
         */
        public static void initPreferences() {
            Preferences.initDouble("ArmPieceCollection", PIECE_COLLECTION_STATIC);
            Preferences.initDouble("ArmShelfPickup", SHELF_PICKUP_STATIC);
            Preferences.initDouble("ArmShelfPickupAlt", SHELF_PICKUP_STATIC);
            Preferences.initDouble("ArmUprightCone", UPRIGHT_CONE_STATIC);

            Preferences.initDouble("ArmTopCone", NODE_POSITIONS_STATIC[0]);
            Preferences.initDouble("ArmTopCube", NODE_POSITIONS_STATIC[1]);
            Preferences.initDouble("ArmMiddleCone", NODE_POSITIONS_STATIC[2]);
            Preferences.initDouble("ArmMiddleCube", NODE_POSITIONS_STATIC[3]);
        }

        public static double pieceCollection() {
            return Preferences.getDouble("ArmPieceCollection", PIECE_COLLECTION_STATIC);
        }

        public static double uprightCone() {
            return Preferences.getDouble("ArmUprightCone", UPRIGHT_CONE_STATIC);
        }

        public static double shelfPickup() {
            return Preferences.getDouble("ArmShelfPickup", SHELF_PICKUP_STATIC);
        }

        public static double shelfPickupAlt() {
            return Preferences.getDouble("ArmShelfPickupAlt", SHELF_PICKUP_ALT_STATIC);
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

        public static double topNode() {
            return Preferences.getDouble("ArmTopCone", NODE_POSITIONS_STATIC[0]);
        }

        public static double middleNode() {
            return Preferences.getDouble("ArmMiddleCone", NODE_POSITIONS_STATIC[2]);
        }
    }

    /**
     * Calibrations for the Elevator Subsystem.
     * 
     * <p> Units of Position is Rot of the drum
     */
    public static final class ElevatorCalibrations {

        public static final double KS = 2.0;
        public static final double KV = 0.86;
        public static final double KA = 0.0;
        public static final double KG = 0.925;

        public static final double KP = 15.0;
        public static final double KI = 0.0;
        public static final double KD = 1.0;

        public static final double MAX_POSITION = 5.5;
        public static final double MAX_VELOCITY_UP = 7.0;
        public static final double MAX_ACCELERATION_UP = 160.0 * 0.5;
        public static final double MAX_VELOCITY_DOWN = 5.0;
        public static final double MAX_ACCELERATION_DOWN = 40.0;
        
        public static final double ARM_CLEARANCE = 35.0;
        
        public static final double TOLERANCE = 0.15;
        
        public static final double POSITION_RETRACTED = 0.0;
        
        public static final double ELEVATOR_DRIVER_SPEED = 4.0 / 50.0;
        
        private static final double PIECE_COLLECTION_STATIC = 2.41;
        private static final double UPRIGHT_CONE_STATIC = 2.475;
        private static final double SHELF_PICKUP_STATIC = 3.53;
        private static final double SHELF_PICKUP_ALT_STATIC = 3.53;
        public static final double[] NODE_POSITIONS_STATIC = { 5.12, 5.08, 2.6, 3.25 };

        /**
         * Initializes the preset arm positions into the preferences tables if the value is not initialized.
         */
        public static void initPreferences() {
            Preferences.initDouble("ElevatorPieceCollection", PIECE_COLLECTION_STATIC);
            Preferences.initDouble("ElevatorShelfPickup", SHELF_PICKUP_STATIC);
            Preferences.initDouble("ElevatorShelfPickupAlt", SHELF_PICKUP_ALT_STATIC);
            Preferences.initDouble("ElevatorUprightCone", UPRIGHT_CONE_STATIC);

            Preferences.initDouble("ElevatorTopCone", NODE_POSITIONS_STATIC[0]);
            Preferences.initDouble("ElevatorTopCube", NODE_POSITIONS_STATIC[1]);
            Preferences.initDouble("ElevatorMiddleCone", NODE_POSITIONS_STATIC[2]);
            Preferences.initDouble("ElevatorMiddleCube", NODE_POSITIONS_STATIC[3]);
        }

        public static double pieceCollection() {
            return Preferences.getDouble("ElevatorPieceCollection", PIECE_COLLECTION_STATIC);
        }

        public static double uprightCone() {
            return Preferences.getDouble("ElevatorUprightCone", PIECE_COLLECTION_STATIC);
        }

        public static double shelfPickup() {
            return Preferences.getDouble("ElevatorShelfPickup", SHELF_PICKUP_STATIC);
        }

        public static double shelfPickupAlt() {
            return Preferences.getDouble("ElevatorShelfPickupAlt", SHELF_PICKUP_ALT_STATIC);
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

        public static double topNode() {
            return Preferences.getDouble("ElevatorTopCone", NODE_POSITIONS_STATIC[0]);
        }

        public static double middleNode() {
            return Preferences.getDouble("ElevatorMiddleCone", NODE_POSITIONS_STATIC[2]);
        }
    }

    /**
     * The Calibrations for the Manipulator Subsystem.
     */
    public static final class ManipulatorCalibrations {
        public static final double INTAKE_SPEED = -0.8;
        public static final double OUTTAKE_SPEED = 0.5;
        public static final double HOLD_SPEED = -0.1;
        
        public static final double PIECE_DETECTION_CURRENT = 30.0;
        public static final double PIECE_DETECTION_RPM = 500.0;

        public static final Color CONE = new Color(0.367, 0.555, 0.086);
        public static final Color CUBE = new Color(0.211, 0.312, 0.470);
        public static final double MATCHER_CONFIDENCE = 0.92;

    }

}
