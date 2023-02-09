package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The class that declares all of the constants.
 */
public class Constants {

    /**
     * Constants for the Swerve Modules.
     */
    public static class SwerveConstants {

        public static final boolean DEBUG = true;

        public static final double TURN_GEAR_RATIO = 12.8;
        public static final double DRIVE_GEAR_RATIO = 8.16;

        // These thing could be tested and experimental measured, but currently they are
        // theoretical.
        public static final double WHEEL_DIAMETER_INCHES = 4;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units
                .inchesToMeters(Math.PI * SwerveConstants.WHEEL_DIAMETER_INCHES);
        public static final double MAX_SPEED_METER_PER_SECOND = 4.0;

        public static final double TRACK_WIDTH_METER = Units.inchesToMeters(19.5);
        public static final double WHEEL_BASE_METER = Units.inchesToMeters(19.5);

        public static final String[] LABELS = new String[] { "Front Left", "Front Right", "Rear Left", "Rear Right" };
        public static final int[] TURN_CAN_IDS = new int[] { 13, 2, 14, 1 };
        public static final int[] DRIVE_CAN_IDS = new int[] { 12, 3, 15, 20 };
        public static final int[] ABS_ENCODER_DIO_PORT = new int[] { 24, 8, 5, 2 };
        public static final boolean[] DRIVE_ENCODER_REVERSED = new boolean[] { false, true, false, true };
        public static final Translation2d[] POSITIONS = new Translation2d[] {
            new Translation2d(WHEEL_BASE_METER / 2, TRACK_WIDTH_METER / 2),
            new Translation2d(WHEEL_BASE_METER / 2, -TRACK_WIDTH_METER / 2),
            new Translation2d(-WHEEL_BASE_METER / 2, TRACK_WIDTH_METER / 2),
            new Translation2d(-WHEEL_BASE_METER / 2, -TRACK_WIDTH_METER / 2) };

    }

    /**
     * Declares the constants for the hardware.
     */
    public static final class Hardware {

        public static final int CTRE_MAG_ENCODER_CPR = 2048;
        public static final int NEO_CPR = 42;

    }

    /**
     * Sets the User Input constants.
     */
    public static class DriverConstants {

        public static final double MAX_STRAFE_SPEED = 3;
        public static final double MAX_TURN_SPEED = 1 * Math.PI;

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double CONTROLLER_DEADBAND = 0.1;

    }

    /**
     * The Constants of the Elevator.
     */
    public static class ElevatorConstants {

        public static final int LEFT_MOTOR_CANID = 0;
        public static final int RIGHT_MOTOR_CANID = 0;

    }

    /**
     * The constants of th manipulator (wrist included).
     */
    public static class ManipulatorConstants {

        public static final int WRIST_MOTOR_CANID = 0;
        public static final double WRIST_MOTOR_SPEED = 1;
        public static final int MANIPULATOR_MOTOR_CANID = 0;
        public static final double MANIPULATOR_MOTOR_SPEED = 1;
        
    }
}
