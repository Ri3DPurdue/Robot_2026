package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.ctre.TalonFXIOSim;
import frc.lib.io.motor.setpoints.IdleSetpoint;
import frc.lib.io.motor.setpoints.VoltageSetpoint;
import frc.lib.mechanismSim.RollerSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.ConfigUtil;
import frc.robot.IDs;
import frc.robot.Robot;

public class FeederConstants {
    // Gearing is a 64 to 12 reduction followed by a 5 to 1 reduction
    public static final double gearing = 1.0;

    // Notable points for system
    public static final Voltage feedVoltage = Volts.of(8.0);
    
    // Setpoints for notable points
    public static final VoltageSetpoint feedSetpoint = new VoltageSetpoint(feedVoltage);
    public static final IdleSetpoint idleSetpoint = new IdleSetpoint();
    
    // Information about motors driving system
    public static final DCMotor motor = DCMotor.getKrakenX60Foc(1);
    
    /**
     *  Gets the final component for the system
     */ 
    public static final MotorComponent<TalonFXIO> getComponent() {
        TalonFXIO io = getMotorIO();
        io.overrideLoggedUnits(Rotations, RPM, Celsius);
        return new MotorComponent<>(io); 
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.INDEXER_FEEDER.id,
                IDs.INDEXER_FEEDER.bus,
                getMainConfig()
                )
            : new TalonFXIOSim(
                IDs.INDEXER_FEEDER.id,
                IDs.INDEXER_FEEDER.bus,
                getMainConfig(),
                getSimObject(),
                gearing
            );
    }

    /** 
     * Get the configuration of the main motor
     */ 
    public static final TalonFXConfiguration getMainConfig() {
        TalonFXConfiguration config = ConfigUtil.getSafeFXConfig(gearing);

        return config;    
    }

    public static final SimObject getSimObject() {
        FlywheelSim system = 
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    motor, 
                    0.01, 
                    gearing),
                motor
            );
        return new RollerSim(system);
    }
}