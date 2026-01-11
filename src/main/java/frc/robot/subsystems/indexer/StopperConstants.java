package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.component.HomingServoMotorComponent;
import frc.lib.component.HomingServoMotorComponent.HomingConfig;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.ctre.TalonFXIOSim;
import frc.lib.io.motor.setpoints.PositionSetpoint;
import frc.lib.mechanismSim.PivotSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.ConfigUtil;
import frc.robot.IDs;
import frc.robot.Robot;

public class StopperConstants {
    // Epsilon threshold is distance that is considered "close" for internal methods and wait commands. Lower value is higher required accuracy
    public static final Angle epsilonThreshold = Units.Degrees.of(10.0);
    
    // Gearing is a 64 to 12 reduction followed by a 5 to 1 reduction
    public static final double gearing = (64.0 / 12.0) * (5.0 / 1.0);
    
    // Constraints of the system's movement (hard stops, potential interferences, soft limits, etc.)
    public static final Angle minAngle = Units.Degrees.of(0.0);
    public static final Angle maxAngle = Units.Degrees.of(90.0);

    // Notable points for system
    public static final Angle openAngle = minAngle;
    public static final Angle blockAngle = maxAngle;
    
    // Setpoints for notable points
    public static final PositionSetpoint openSetpoint = new PositionSetpoint(openAngle);
    public static final PositionSetpoint blockSetpoint = new PositionSetpoint(blockAngle);
    
    // Information about motors driving system
    public static final DCMotor motor = DCMotor.getKrakenX60Foc(1);
    
    /**
     *  Gets the final component for the system
     */ 
    public static final HomingServoMotorComponent<TalonFXIO> getComponent() {
        TalonFXIO io = getMotorIO();
        io.overrideLoggedUnits(Degrees, DegreesPerSecond, Celsius);
        return new HomingServoMotorComponent<TalonFXIO>(io, epsilonThreshold, openAngle, getHomingConfig()); 
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.INDEXER_STOPPER.id,
                IDs.INDEXER_STOPPER.bus,
                getMainConfig()
                )
            : new TalonFXIOSim(
                IDs.INDEXER_STOPPER.id,
                IDs.INDEXER_STOPPER.bus,
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
        ConfigUtil.withSoftLimits(config, maxAngle, minAngle);
        config.Slot0.kP = 5.0;
        config.Slot0.kD = 0.0;

        return config;    
    }

    /**
     * Gets the homing configuration for the component
     */
    public static final HomingConfig getHomingConfig() {
        HomingConfig config = new HomingConfig();

        config.homePosition = openAngle;
        config.homeSetpoint = openSetpoint;
        config.homingDebouce = Seconds.of(0.1);
        config.homingVelocity = DegreesPerSecond.of(1.0);
        config.homingVoltage = Volts.of(-2.0);

        return config;
    }

    public static final SimObject getSimObject() {
        SingleJointedArmSim system = 
            new SingleJointedArmSim(
                motor, 
                gearing, 
                0.01, 
                0.2, 
                minAngle.in(Units.Radians), 
                maxAngle.in(Units.Radians), 
                false,
                openAngle.in(Units.Radians), 
                0.0, 0.0);
        return new PivotSim(system);
    }
}