package frc.robot.subsystems;

import frc.robot.controlBoard.ControlBoardConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Superstructure implements Loggable {
    public final Drive drive = new Drive();
    public final Vision vision = new Vision(drive::addVisionMeasurement);
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Shooter shooter = new Shooter();
    public final Climber climber = new Climber();


    public Superstructure() {
        NamedCommands.registerCommand("Intake", intake());
        NamedCommands.registerCommand("Stop Intaking", intake.stow());
        NamedCommands.registerCommand("Prep Shooting", prepHubShot());
        NamedCommands.registerCommand("Shoot", shoot());
        NamedCommands.registerCommand("Stow", stow());
    }

    @Override
    public void log(String path) {
        Logger.log(path, "Drive", drive);
        Logger.log(path, "Vision", vision);
        Logger.log(path, "Intake", intake);
        Logger.log(path, "Indexer", indexer);
        Logger.log(path, "Shooter", shooter);
        Logger.log(path, "Climber", climber);
    }

    public Command intake() {
        return Commands.parallel(
            indexer.intake(),
            intake.intake()
        ).andThen(Commands.idle());
    }

    public Command prepShot(Supplier<Pose2d> targetPose) {
        return Commands.parallel(
            shooter.prepVariableShot(() -> drive.getShotDistance(targetPose.get().getTranslation())),
            drive.alignDrive(ControlBoardConstants.driver, targetPose)
        );
    }

    public Command prepFerryShot() {
        return prepShot(() -> DriveConstants.getFerryPose().toPose2d());
    } 

    public Command prepHubShot() {
        return prepShot(() -> DriveConstants.getHubPose().toPose2d());
    }

    public Command stow() {
        return Commands.parallel(
            intake.stow(),
            indexer.idle(),
            shooter.idleMotors(),
            climber.stow()
        );
    }

    public Command shoot() {
        return indexer.feed();
    }

    public Command spit() {
        return Commands.parallel(
            intake.spit(),
            indexer.spit(),
            Commands.idle()
        );
    }
    
}
