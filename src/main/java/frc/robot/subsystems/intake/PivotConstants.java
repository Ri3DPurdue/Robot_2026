package frc.robot.subsystems.intake;

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

import static edu.wpi.first.units.Units.*;

public class PivotConstants {
    public static final Angle epsilonThreshold = Units.Degrees.of(5.0);
    
    // TODO: Update gearing
    public static final double gearing = 5.; 
    
    // TODO: Update limits
    public static final Angle minAngle = Units.Degrees.of(-10.0);
    public static final Angle maxAngle = Units.Degrees.of(110.0);

    // TODO: Find actual positions
    public static final Angle intakeAngle = Units.Degrees.of(-3.0);
    public static final Angle stowAngle = maxAngle;
    public static final Angle spitAngle = Units.Degrees.of(0.0);
    
    public static final PositionSetpoint intakeSetpoint = new PositionSetpoint(intakeAngle);
    public static final PositionSetpoint stowSetpoint = new PositionSetpoint(stowAngle);
    public static final PositionSetpoint spitSetpoint = new PositionSetpoint(spitAngle);

    public static final DCMotor motor = DCMotor.getKrakenX60(1); 

    /**
     * Gets the pivot with a built-in homing sequence
     */
    public static final HomingServoMotorComponent<TalonFXIO> getComponent() {
        TalonFXIO io = getMotorIO();
        io.overrideLoggedUnits(Degrees, DegreesPerSecond, Celsius);
        return new HomingServoMotorComponent<TalonFXIO>(io, epsilonThreshold, stowAngle, getHomingConfig());     
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.INTAKE_PIVOT.id,
                IDs.INTAKE_PIVOT.bus,
                getMainConfig()
                )
            : new TalonFXIOSim(
                IDs.INTAKE_PIVOT.id,
                IDs.INTAKE_PIVOT.bus,
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
        // TODO: Tune kP and kD
        config.Slot0.kP = 10.0;
        config.Slot0.kD = 10.0;

        return config;    
    }

    /**
     * Gets the homing configuration for the component
     */
    public static final HomingConfig getHomingConfig() {
        HomingConfig config = new HomingConfig();

        config.homePosition = stowAngle;
        config.homeSetpoint = stowSetpoint;
        config.homingDebouce = Seconds.of(0.1);
        config.homingVelocity = DegreesPerSecond.of(1.0);
        config.homingVoltage = Volts.of(2.0);

        return config;
    }

    /**
     * Gets an object to represent the system when running simulation
     */
    public static final SimObject getSimObject() {
        // TODO: Update these with actual characteristics of the system
        SingleJointedArmSim system = 
            new SingleJointedArmSim(
                motor, 
                gearing, 
                0.2, 
                0.3, 
                minAngle.in(Radians), 
                maxAngle.in(Radians), 
                false,
                stowAngle.in(Radians), 
                0.0, 0.0);
        return new PivotSim(system);
    }
}
