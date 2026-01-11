package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.setpoints.VelocitySetpoint;

public class Shooter extends ComponentSubsystem {
    private final FlywheelMotorComponent<TalonFXIO> topflywheel;
    private final FlywheelMotorComponent<TalonFXIO> bottomflywheel;


    public Shooter() {
        topflywheel = registerComponent("Top Flywheel", TopFlywheelConstants.getComponent());
        bottomflywheel = registerComponent("Bottom Flywheel", BottomFlywheelConstants.getComponent());
    }

    public Command idleMotors(){
        return withRequirement(
            Commands.parallel(
                topflywheel.applySetpointCommand(TopFlywheelConstants.idleSetpoint),
                bottomflywheel.applySetpointCommand(BottomFlywheelConstants.idleSetpoint)
            )
        );
    }

    public Command lowVoltage(){
        return withRequirement(
            Commands.parallel(
                topflywheel.applySetpointCommand(TopFlywheelConstants.lowVoltageSetpoint),
                bottomflywheel.applySetpointCommand(BottomFlywheelConstants.lowVoltageSetpoint)
            )
        );
    }

    public Command prepBasicShot() {
        return withRequirement(
            Commands.parallel(
                topflywheel.applySetpointCommand(TopFlywheelConstants.shotSetpoint),
                bottomflywheel.applySetpointCommand(BottomFlywheelConstants.shotSetpoint)
            )
        );
    }

    public Command prepVariableShot(Supplier<Distance> shotDistanceSupplier) {
        return withRequirement(
            Commands.parallel(
                topflywheel.followSetpointCommand(() -> new VelocitySetpoint(TopFlywheelConstants.shotDistanceVelocityMap.get(shotDistanceSupplier.get()))),
                bottomflywheel.followSetpointCommand(() -> new VelocitySetpoint(BottomFlywheelConstants.shotDistanceVelocityMap.get(shotDistanceSupplier.get())))
            )
        );
    }
    
}