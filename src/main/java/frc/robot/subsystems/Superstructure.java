package frc.robot.subsystems;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Superstructure implements Loggable {
    public final Drive drive = new Drive();
    public final Shooter shooter = new Shooter();

    @Override
    public void log(String path) {
        Logger.log(path, "Drive", drive);
        Logger.log(path, "Shooter", shooter);
    }
}
