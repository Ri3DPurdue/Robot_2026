package frc.robot.controlBoard;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        driver.start().onTrue(Commands.runOnce(() -> {
            s.drive.resetRotation(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Rotation2d.k180deg : Rotation2d.kZero);
        }));

        driver.y().onTrue(s.spit().andThen(Commands.idle()));
        driver.b().onTrue(s.stow());
        driver.x().whileTrue(s.prepHubShot());
        driver.a().whileTrue(s.prepFerryShot());
        driver.leftTrigger(ControlBoardConstants.triggerThreshold).whileTrue(s.intake());
        driver.rightTrigger(ControlBoardConstants.triggerThreshold).whileTrue(s.shoot());

        // driver.povUp().whileTrue(s.climber.fullExtend());
        // driver.povLeft().whileTrue(s.climber.partialExtend());
        // driver.povRight().whileTrue(s.climber.pull());
        // driver.povDown().whileTrue(s.climber.stow());

        driver.povUp().onTrue(s.shooter.prepBasicShot());

    }
}
