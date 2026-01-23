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
        setDefaultCommand(stow().andThen(Commands.idle()));
        climber.disable();
    }

    public Command fullExtend() {
        return withRequirement(
            climber.applyPositionSetpointCommandWithWait(ClimberConstants.fullExtendSetpoint)
        );
    }

    public Command partialExtend() {
        return withRequirement(
            climber.applyPositionSetpointCommandWithWait(ClimberConstants.partialExtendSetpoint)
        );
    }

    public Command pull() {
        return withRequirement(
            climber.applyPositionSetpointCommandWithWait(ClimberConstants.pullSetpoint)
        );
    }

    public Command stow() {
        return withRequirement(
            climber.applyPositionSetpointCommandWithWait(ClimberConstants.stowSetpoint)
        );
    }
}
