package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;

public class Climber extends ComponentSubsystem {
    private final ServoMotorComponent<TalonFXIO> climber;

    public Climber() {
        climber = registerComponent("Climber", ClimberConstants.getComponent());
    }

    public Command extend() {
        return withRequirement(
            Commands.print("Climber::extend: Not yet implemented")
        );
    }

    public Command pull() {
        return withRequirement(
            Commands.print("Climber::pull: Not yet implemented")
        );
    }

    public Command stow() {
        return withRequirement(
            Commands.print("Climber::stow: Not yet implemented")
        );
    }
}
