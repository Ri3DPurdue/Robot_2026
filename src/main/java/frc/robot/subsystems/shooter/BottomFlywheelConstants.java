package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.component.FlywheelMotorComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.ctre.TalonFXIOSim;
import frc.lib.io.motor.setpoints.IdleSetpoint;
import frc.lib.io.motor.setpoints.VelocitySetpoint;
import frc.lib.io.motor.setpoints.VoltageSetpoint;
import frc.lib.mechanismSim.RollerSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.ConfigUtil;
import frc.robot.IDs;
import frc.robot.Robot;

public class BottomFlywheelConstants {
    // Epsilon threshold is velocity that is considered "close" for internal methods and wait commands. Lower value is higher required accuracy
    public static final AngularVelocity epsilonThreshold = Units.RPM.of(100); //TODO get actual threshold
    
    // Gearing is a 48 to 40 reduction
    public static final double gearing = 1.0; //TODO get actual gearing

    // Notable points for system
    public static final AngularVelocity shotVelocity = Units.RPM.of(2000.0); //TODO get actual velocity
    public static final Voltage unjamVoltage = Units.Volts.of(-8.0); //TODO get actual voltage
    public static final Voltage lowVoltage = Units.Volts.of(1); //TODO get actual voltage


    // Setpoints for notable points
    public static final VelocitySetpoint shotSetpoint = new VelocitySetpoint(shotVelocity);
    public static final VoltageSetpoint feedSetpoint = new VoltageSetpoint(unjamVoltage);
    public static final IdleSetpoint idleSetpoint = new IdleSetpoint();
    public static final VoltageSetpoint lowVoltageSetpoint = new VoltageSetpoint(lowVoltage);


    // Information about motors driving system
    public static final DCMotor motor = DCMotor.getKrakenX60(2); // Only needed for sim

    /**
     *  Gets the final component for the system
     */ 
    public static final FlywheelMotorComponent<TalonFXIO> getComponent() {
        TalonFXIO io =  getMotorIO();
        io.overrideLoggedUnits(Degrees, DegreesPerSecond, Celsius);
        return new FlywheelMotorComponent<TalonFXIO>(getMotorIO(), epsilonThreshold);
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.SHOOTER_BOTTOM_FLYWHEEL.id,
                IDs.SHOOTER_BOTTOM_FLYWHEEL.bus,
                getMainConfig()
                )
            : new TalonFXIOSim(
                IDs.SHOOTER_BOTTOM_FLYWHEEL.id,
                IDs.SHOOTER_BOTTOM_FLYWHEEL.bus,
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
        config.Slot0.kP = 1.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.15;

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