package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.DigitalIOComponent;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;

public class Indexer extends ComponentSubsystem {
    private final MotorComponent<TalonFXIO> belt;
    private final MotorComponent<TalonFXIO> stopper;
    private final DigitalIOComponent beamBreak;


    public Indexer() {
        belt = registerComponent("Belt", BeltConstants.getComponent());
        stopper = registerComponent("Feeder", FeederConstants.getComponent());
        beamBreak = registerComponent("Beam Break", BeamBreakConstants.getComponent());

    }

    public Command intake() {
        return withRequirement(
            Commands.parallel(
                belt.applySetpointCommand(BeltConstants.intakeSetpoint),
                Commands.sequence(
                    stopper.applySetpointCommand(FeederConstants.intakeSetpoint)
                        .onlyWhile(beamBreak::getDebounced)
                        .onlyIf(beamBreak::getDebounced),
                    stopper.applySetpointCommand(FeederConstants.idleSetpoint)
                )
            )
        );
    }

    public Command idle() {
        return withRequirement(
            Commands.parallel(
                belt.applySetpointCommand(BeltConstants.idleSetpoint),
                stopper.applySetpointCommand(FeederConstants.idleSetpoint)
            )
        );    
    }

    public Command feed() {
        return withRequirement(
            Commands.parallel(
                belt.applySetpointCommand(BeltConstants.feedSetpoint),
                stopper.applySetpointCommand(FeederConstants.feedSetpoint)
            )
        );    
    }

    public Command spit() {
        return withRequirement(
            Commands.parallel(
                belt.applySetpointCommand(BeltConstants.spitSetpoint),
                stopper.applySetpointCommand(FeederConstants.spitSetpoint)
            )
        );    
    }
}
