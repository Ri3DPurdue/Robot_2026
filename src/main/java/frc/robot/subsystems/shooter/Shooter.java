package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;

public class Shooter extends ComponentSubsystem {
    private final FlywheelMotorComponent<TalonFXIO> Topflywheel;
    private final FlywheelMotorComponent<TalonFXIO> Bottomflywheel;


    public Shooter() {
        Topflywheel = registerComponent("Top Flywheel", TopFlywheelConstants.getComponent());
        Bottomflywheel = registerComponent("Bottom Flywheel", BottomFlywheelConstants.getComponent());
    }

    public Command idleMotors(){
        return withRequirement(
            Commands.parallel(
                Topflywheel.applySetpointCommand(TopFlywheelConstants.idleSetpoint),
                Bottomflywheel.applySetpointCommand(BottomFlywheelConstants.idleSetpoint)
            )
        );
    }

    public Command lowVoltage(){
        return withRequirement(
            Commands.parallel(
                Topflywheel.applySetpointCommand(TopFlywheelConstants.lowVoltageSetpoint),
                Bottomflywheel.applySetpointCommand(BottomFlywheelConstants.lowVoltageSetpoint)
            )
        );
    }

    public Command prepShot() {
        return withRequirement(
            Commands.parallel(
                Topflywheel.applySetpointCommand(TopFlywheelConstants.shotSetpoint),
                Bottomflywheel.applySetpointCommand(BottomFlywheelConstants.shotSetpoint)
            )
        );
    }
    
}