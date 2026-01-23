package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.DigitalIOComponent;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;

public class Indexer extends ComponentSubsystem {
    private final MotorComponent<TalonFXIO> belt;
    private final MotorComponent<TalonFXIO> feeder;
    private final DigitalIOComponent beamBreak;


    public Indexer() {
        belt = registerComponent("Belt", BeltConstants.getComponent());
        feeder = registerComponent("Feeder", FeederConstants.getComponent());
        beamBreak = registerComponent("Beam Break", BeamBreakConstants.getComponent());
        setDefaultCommand(idle().andThen(Commands.idle()));
    }

    public Command intake() {
        return withRequirement(
            Commands.parallel(
                belt.applySetpointCommand(BeltConstants.intakeSetpoint),
                feeder.applySetpointCommand(FeederConstants.idleSetpoint)
                
            )
        );
    }

    public Command idle() {
        return withRequirement(
            Commands.parallel(
                belt.applySetpointCommand(BeltConstants.idleSetpoint),
                feeder.applySetpointCommand(FeederConstants.idleSetpoint)
            )
        );    
    }

    public Command feed() {
        return withRequirement(
            Commands.parallel(
                belt.applySetpointCommand(BeltConstants.feedSetpoint),
                feeder.applySetpointCommand(FeederConstants.feedSetpoint)
            )
        );    
    }

    public Command spit() {
        return withRequirement(
            Commands.parallel(
                belt.applySetpointCommand(BeltConstants.spitSetpoint),
                feeder.applySetpointCommand(FeederConstants.spitSetpoint)
            )
        );    
    }
}
