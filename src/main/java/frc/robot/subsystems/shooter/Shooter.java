package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.DigitalIOComponent;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.component.MotorComponent;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;

public class Shooter extends ComponentSubsystem {
    private final FlywheelMotorComponent<TalonFXIO> Topflywheel;
    private final FlywheelMotorComponent<TalonFXIO> Bottomflywheel;


    public Shooter() {
        Topflywheel = registerComponent("TopFlywheel", TopFlywheelConstants.getComponent());
        Bottomflywheel = registerComponent("BottomFlywheel", BottomFlywheelConstants.getComponent());
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

    public Command prepShot(){
        return withRequirement(
            Topflywheel.applyVelocitySetpointCommandWithWait(TopFlywheelConstants.shotSetpoint)
        );
    }

    public Command fire() {
        return withRequirement(
            Commands.parallel(
                Topflywheel.applySetpointCommand(TopFlywheelConstants.shotSetpoint),
                Bottomflywheel.applySetpointCommand(BottomFlywheelConstants.shotSetpoint)
            )
        );
    }

    public Command fireWhenReady() {
        return withRequirement(
            Commands.sequence(
                prepShot(),
                fire()
            )
        );
    }


    

    
}