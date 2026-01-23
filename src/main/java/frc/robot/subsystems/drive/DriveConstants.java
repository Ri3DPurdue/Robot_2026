package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.logging.Logger;

public class DriveConstants {
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double maxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    public static final Distance shooterSideOffset = Units.Inches.of(6.0);

    public static final Transform2d shooterTransform = new Transform2d(Units.Inches.of(0.0), shooterSideOffset, new Rotation2d());

    public static final Pose3d redHubPose = new Pose3d(Units.Inches.of(468.56), Units.Inches.of(158.32), Units.Inches.of(72.0), new Rotation3d());
    public static final Pose3d blueHubPose = new Pose3d(Units.Inches.of(152.56), Units.Inches.of(158.32),  Units.Inches.of(72.0), new Rotation3d());

    public static final Pose3d redFerryPoseDepot = new Pose3d(14.3, 6, 0, Rotation3d.kZero);
    public static final Pose3d redFerryPoseOutpost = new Pose3d(14.3, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseDepot = new Pose3d(2.1, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseOutpost = new Pose3d(2.1, 6, 0, Rotation3d.kZero);

    public static final Angle epsilonAngleToGoal = Degrees.of(1.0);

    public static final Pose3d getHubPose() {
        Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
        Logger.log("HUB POSE", pose);
        return pose;
    }

    public static final Pose3d getFerryPose(Translation2d robotPose) {
        if(DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            if (robotPose.getDistance(redFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose.getDistance(redFerryPoseOutpost.getTranslation().toTranslation2d())) {
                return redFerryPoseOutpost;
            } else {
                return redFerryPoseDepot;
            }
        } else {
            if (robotPose.getDistance(blueFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose.getDistance(blueFerryPoseOutpost.getTranslation().toTranslation2d())) {
                return blueFerryPoseOutpost;
            } else {
                return blueFerryPoseDepot;
            }
        }
    }

    public static final PIDController rotationController = getRotationController();

    private static final PIDController getRotationController() {
        PIDController controller = new PIDController(1.5, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }
    
}
