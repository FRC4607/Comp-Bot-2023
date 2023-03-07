package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The class that declares all of the constants.
 */
public class Constants {

    /**
     * Constants for the Swerve Drivetrain.
     */
    public static class SwerveConstants {

        public static final boolean DEBUG = false;

        public static final double TURN_GEAR_RATIO = (32.0 / 15.0) *  (6.0); // MK4
        public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // MK4 L2

        // These thing could be tested and experimental measured, but currently they are
        // theoretical.
        public static final double WHEEL_DIAMETER_INCHES = 4;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units
                .inchesToMeters(Math.PI * SwerveConstants.WHEEL_DIAMETER_INCHES);
        public static final double TRACK_WIDTH_METER = Units.inchesToMeters(19.5);
        public static final double WHEEL_BASE_METER = Units.inchesToMeters(19.5);

        public static final String[] LABELS = new String[] { "Front Left", "Front Right", "Rear Left", "Rear Right" };
        public static final int[] TURN_CAN_IDS = new int[] { 17, 19, 2, 40 };
        public static final int[] DRIVE_CAN_IDS = new int[] { 16, 18, 3, 1 };
        public static final boolean[] DRIVE_ENCODER_REVERSED = new boolean[] { false, true, false, true };
        public static final Translation2d[] POSITIONS = new Translation2d[] {
            new Translation2d(WHEEL_BASE_METER / 2, TRACK_WIDTH_METER / 2),
            new Translation2d(WHEEL_BASE_METER / 2, -TRACK_WIDTH_METER / 2),
            new Translation2d(-WHEEL_BASE_METER / 2, TRACK_WIDTH_METER / 2),
            new Translation2d(-WHEEL_BASE_METER / 2, -TRACK_WIDTH_METER / 2) };

        public static final int PIGEON2_CAN_ID = 23;

        public static final double GYRO_RECALIBRATION_TIME = 120.0;

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

        public static final int ELEVATOR_MOTOR_CAN_ID = 4;

        public static final int ABSOLUTE_ENCODER_PORT = 0;
        public static final int ENCODER_A_PORT = 1;
        public static final int ENCODER_B_PORT = 2;
        public static final int ENCODER_I_PORT = 3;

        public static final double GEAR_RATIO_MOTOR = 9.0 * 48.0 / 30.0;
        public static final double GEAR_RATIO_ENCODER = 48.0 / 30.0;

    }

    /**
     * The constants of the manipulator.
     */
    public static class ManipulatorConstants {

        public static final int MANIPULATOR_MOTOR_CAN_ID = 14;
        public static final double MANIPULATOR_MOTOR_SPEED = 0.75;
        
    }

    /**
     * The constants of the Arm.
     */
    public static class ArmConstants {
        public static final int ARM_MOTOR_CAN_ID = 15;
    }
}
