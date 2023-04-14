package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This thread runs separately from the main thread and will periodically check the AprilTag status. 
 * If a new AprilTag is found, it will acquire a lock and update the pose estimator.
 */
public class LimelightThread implements Runnable {
    private Pose2d m_lastPose2d = new Pose2d();
    private final DrivetrainSubsystem m_dt;

    public LimelightThread(DrivetrainSubsystem dt) {
        m_dt = dt;
    }
    
    @Override
    public void run() {
        while (true) {
            if (LimelightHelpers.getCurrentPipelineIndex("limelight") == 0
                && LimelightHelpers.getTV("limelight")) {
                Pose2d currentAT = DriverStation.getAlliance() == DriverStation.Alliance.Red 
                    ? LimelightHelpers.getBotPose2d_wpiRed("limelight") 
                    : LimelightHelpers.getBotPose2d_wpiBlue("limelight");
                if (!m_lastPose2d.equals(currentAT)) {
                    m_lastPose2d = currentAT;
                    double tl = LimelightHelpers.getLatency_Pipeline("limelight");
                    double cl = LimelightHelpers.getLatency_Capture("limelight");
                    double ts = Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);
                    m_dt.synchronizedVisionUpdate(currentAT, ts);
                }
            }
            try {
                Thread.sleep(25); // 25 ms delay
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Starts execution of the thread and returns a Thread object corresponding to it.

     * @return A Thread object corresponding to the thread this method starts.
     */
    public Thread start() {
        Thread t = new Thread(this, "LLThread");
        t.start();
        return t;
    }
    
}
