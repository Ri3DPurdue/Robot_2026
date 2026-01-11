package frc.robot.subsystems;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Superstructure implements Loggable {
    public final Drive drive = new Drive();
    public final Climber climber = new Climber();

    @Override
    public void log(String path) {
        Logger.log(path, "Drive", drive);
    }
}
