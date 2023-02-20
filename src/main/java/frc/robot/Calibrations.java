package frc.robot;

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

        /** Max acceleration in Meters / s. Experimentally determined at voltage of 9 volts. */
        public static final double MAX_SPEED_METER = 3.39;  
        /** Max acceleration in Meters / s^2. Experimentally determined with a step voltage of 9 volts. */
        public static final double MAX_ACCELERATION = 4.39;

    }

    /**
     * Calibrations for the arm subsystems.
     */
    public static final class ArmCalibrations {
        
        public static final double POSITION_PIECE_COLLECTION = 125.0;
        public static final double POSITION_LEVEL = 80.0;
        public static final double POSITION_TOP_NODE = 90.0;
        public static final double POSITION_RETRACTED = 40.0;

        public static final double[] ARM_PLACEMENT_POSITIONS = {72, 70, 68, 70};
        
        
        public static final double ELEVATOR_CLEARANCE = 40.0;

        public static final double KG = 0.5;

        public static final double KP = 0.15;
        public static final double KI = 0.0;
        public static final double KD = 0.0;


        public static final double MAX_VELOCITY = 90.0;
        public static final double MAX_ACCELERATION = 60.0;

        public static final double TOLERANCE = 4.0;
    }

    /**
     * Calibrations for the Elevator Subsystem.
     */
    public static final class ElevatorCalibrations {
        
        public static final double[] ELEVATOR_PLACEMENT_POSITIONS = {92.3, 60, 53, 40};
        
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
    }

    /**
     * The Calibrations for the Manipulator Subsystem.
     */
    public static final class ManipulatorCalibrations {
        public static final double INTAKE_SPEED = -0.8;
        public static final double OUTTAKE_SPEED = 0.5;
        public static final double HOLD_SPEED = -0.2;
        
    }

}
