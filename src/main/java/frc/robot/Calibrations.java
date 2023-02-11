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

}
