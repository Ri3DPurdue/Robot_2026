package frc.robot.subsystems.vision;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;
import frc.robot.subsystems.vision.Limelight.PoseEstimate;

public class Vision extends SubsystemBase implements Loggable{
    private final Limelight limelight;
    private final BiConsumer<Pose2d, Double> visionUpdater;

    public Vision(BiConsumer<Pose2d, Double> addVisionEstimate) {
        limelight = VisionConstants.getLimelight();
        visionUpdater = addVisionEstimate;
    }

    // Returns null if no new estimate since last call of this method
    public PoseEstimate getLatestEstimate() {
        return limelight.getPoseMT1();
    }

    @Override
    public void periodic() {
        addLatestEstimate();
    }

    public boolean shouldAcceptEstimate(PoseEstimate estimate) {
        return estimate.exists();
    }

    public void addLatestEstimate() {
        double currentTimeSeconds = Timer.getFPGATimestamp();
        PoseEstimate latestEstimate = getLatestEstimate();
        if (shouldAcceptEstimate(latestEstimate)) {
            double estimateTimeSeconds = currentTimeSeconds + latestEstimate.latencySeconds();

            visionUpdater.accept(latestEstimate.pose(), estimateTimeSeconds);
        }
    }


    @Override
    public void log(String path) {
        PoseEstimate latestEstimate = getLatestEstimate();
        Logger.log(path, "Latest Estimate/Pose", latestEstimate.pose());
        Logger.log(path, "time", Timer.getFPGATimestamp());
    }
    
}
