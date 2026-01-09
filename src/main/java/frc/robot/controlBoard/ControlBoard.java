package frc.robot.controlBoard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;

public class ControlBoard {
    public static void bindControls(Superstructure s) {
        // Shorter names to avoid long lines of accessing constants
        CommandXboxController driver = ControlBoardConstants.driver;
        CommandXboxController operator = ControlBoardConstants.operator;

        // Controls
        s.drive.setDefaultCommand(s.drive.teleopDrive(driver));
        driver.start().onTrue(Commands.runOnce(() -> s.drive.resetPose(new Pose2d())));

    }
}
