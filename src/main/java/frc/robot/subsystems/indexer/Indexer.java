package frc.robot.subsystems.indexer;

import frc.lib.component.ComponentSubsystem;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;

public class Indexer extends ComponentSubsystem {
    private final MotorComponent<TalonFXIO> belt;

    public Indexer() {
        belt = registerComponent("Belt", BeltConstants.getComponent());
    }
}
