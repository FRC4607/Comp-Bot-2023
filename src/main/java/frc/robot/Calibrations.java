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
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        public static final double DRIVE_FF_KS = (0.125 + 0.108) / 2;
        public static final double DRIVE_FF_KV = (3.23 + 3.25) / 2;
        // Too few data points to calculate. Will revisit if time allows.
        public static final double DRIVE_FF_KA = 0.0;

        public static final double MAX_SPEED_METER_PER_SECOND = 3.0; // Estimate

    }

}
