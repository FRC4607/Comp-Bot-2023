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

        // Smart motion is very jittery and slow
        public static final double TURN_MAX_VELOCITY = 46.1 * 0.8;
        public static final double TURN_MAX_ACCELERATION = 0.180 * 0.8;
        public static final double TURN_MIN_VELOCITY = 0.001;


        // Yet to be tuned
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        public static final double MAX_SPEED_METER_PER_SECOND = 10.0; // Estimate

    }

}
