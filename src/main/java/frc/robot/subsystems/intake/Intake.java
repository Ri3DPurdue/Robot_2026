package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.HomingServoMotorComponent;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;

public class Intake extends ComponentSubsystem {
    private HomingServoMotorComponent<TalonFXIO> pivot;
    private MotorComponent<TalonFXIO> roller;

    public Intake() {
        pivot = registerComponent("Pivot", PivotConstants.getComponent());
        roller = registerComponent("Rollers", RollerConstants.getComponent());
    }

    public Command intake() {
        return withRequirement(
            Commands.sequence(
                pivot.applyPositionSetpointCommandWithWait(PivotConstants.intakeSetpoint),
                roller.applySetpointCommand(RollerConstants.inwardsSetpoint)
            )
        );
    }

    public Command stow() {
        return withRequirement(
            Commands.parallel(
                pivot.applySetpointCommand(PivotConstants.stowSetpoint),
                roller.applySetpointCommand(RollerConstants.idleSetpoint)
            )
        );
    }

    public Command spit() {
        return withRequirement(
            Commands.parallel(
                pivot.applySetpointCommand(PivotConstants.spitSetpoint),
                roller.applySetpointCommand(RollerConstants.spitSetpoint)
            )
        );
    }
}
