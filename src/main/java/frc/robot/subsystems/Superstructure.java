package frc.robot.subsystems;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.Shooter;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Superstructure implements Loggable {
    public final Drive drive = new Drive();
    public final Vision vision = new Vision((A, B) -> {});
    public final Climber climber = new Climber();
    public final Shooter shooter = new Shooter();

    @Override
    public void log(String path) {
        Logger.log(path, "Drive", drive);
        Logger.log(path, "Vision", vision);
        Logger.log(path, "Shooter", shooter);
    }
}
