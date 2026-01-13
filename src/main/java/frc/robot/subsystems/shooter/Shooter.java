package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.setpoints.VelocitySetpoint;
import frc.lib.util.TunableNumber;

public class Shooter extends ComponentSubsystem {
    private final FlywheelMotorComponent<TalonFXIO> topFlywheel;
    private final FlywheelMotorComponent<TalonFXIO> bottomFlywheel;
    private final TunableNumber topNum = new TunableNumber("TOP VEL", 0, true);
    private final TunableNumber botNum = new TunableNumber("BOT VEL", 0, true);


    public Shooter() {
        topFlywheel = registerComponent("Top Flywheel", TopFlywheelConstants.getComponent());
        bottomFlywheel = registerComponent("Bottom Flywheel", BottomFlywheelConstants.getComponent());
        setDefaultCommand(off());
    }

    public Command test() {
        return withRequirement(
            Commands.parallel(
                topFlywheel.followSetpointCommand(() -> new VelocitySetpoint(Units.RPM.of(topNum.get()))),
                bottomFlywheel.followSetpointCommand(() -> new VelocitySetpoint(Units.RPM.of(botNum.get())))
            )
        );
    }

    public Command off(){
        return withRequirement(
            Commands.parallel(
                topFlywheel.applySetpointCommand(TopFlywheelConstants.idleSetpoint),
                bottomFlywheel.applySetpointCommand(BottomFlywheelConstants.idleSetpoint)
            )
        );
    }

    public Command steadyState(){
        return withRequirement(
            Commands.parallel(
                topFlywheel.applySetpointCommand(TopFlywheelConstants.steadyStateSetpoint),
                bottomFlywheel.applySetpointCommand(BottomFlywheelConstants.steadyStateSetpoint)
            )
        );
    }

    public Command prepBasicShot() {
        return withRequirement(
            Commands.parallel(
                topFlywheel.applySetpointCommand(TopFlywheelConstants.shotSetpoint),
                bottomFlywheel.applySetpointCommand(BottomFlywheelConstants.shotSetpoint),
                Commands.idle()
            )
        );
    }

    public Command prepVariableShot(Supplier<Distance> shotDistanceSupplier) {
        return withRequirement(
            Commands.parallel(
                topFlywheel.followSetpointCommand(() -> new VelocitySetpoint(TopFlywheelConstants.shotDistanceVelocityMap.get(shotDistanceSupplier.get()))),
                bottomFlywheel.followSetpointCommand(() -> new VelocitySetpoint(BottomFlywheelConstants.shotDistanceVelocityMap.get(shotDistanceSupplier.get())))
            )
        );
    }
    
}