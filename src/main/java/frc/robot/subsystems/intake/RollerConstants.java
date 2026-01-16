package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
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

import static edu.wpi.first.units.Units.*;

public class RollerConstants {
    public static final double gearing = 42./18;

    // TODO: Find actual voltages
    public static final Voltage intakeVoltage = Units.Volts.of(4.0);
    public static final Voltage spitVoltage = Units.Volts.of(-6.0);

    public static final VoltageSetpoint inwardsSetpoint = new VoltageSetpoint(intakeVoltage);
    public static final VoltageSetpoint spitSetpoint = new VoltageSetpoint(spitVoltage);
    public static final IdleSetpoint idleSetpoint = new IdleSetpoint();

    public static final DCMotor motor = new DCMotor( 
        12.0, 
        4.05, 
        275.0, 
        1.4, 
        edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(7530.0), 
        1 
    );

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    public static final MotorComponent<TalonFXIO> getComponent() {
        TalonFXIO io = getMotorIO();
        io.overrideLoggedUnits(Rotations, RPM, Celsius);
        return new MotorComponent<TalonFXIO>(io);
    }

    @SuppressWarnings("unchecked")
    public static final TalonFXIO getMotorIO() {
        return Robot.isReal() 
            ? new TalonFXIO(
                IDs.INTAKE_ROLLERS.id,
                IDs.INTAKE_ROLLERS.bus,
                getMainConfig()
                )
            : new TalonFXIOSim(
                IDs.INTAKE_ROLLERS.id,
                IDs.INTAKE_ROLLERS.bus,
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
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 1.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.15;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        return config;
    }

    /**
     * Gets an object to represent the system when running simulation
     */
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
