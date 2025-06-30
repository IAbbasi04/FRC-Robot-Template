package lib.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import lib.hardware.motor.ctre.*;
import lib.hardware.motor.rev.*;

/**
 * Utility class for creating and managing motors
 * 
 * TODO - Probably needs testing or fixing but I'm too lazy now to do it
 */
public class MotorFactory {
    private static List<BaseMotor> motors = new ArrayList<>();

    public static TalonFXMotor createDefaultTalonFX(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new TalonFXMotor(config));
        }

        return (TalonFXMotor) motors.get(port);
    }

    public static SparkMaxMotor createDefaultSparkMax(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new SparkMaxMotor(config));
        }

        return (SparkMaxMotor) motors.get(port);
    }

    public static SparkFlexMotor createDefaultSparkFlex(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new SparkFlexMotor(config));
        }

        return (SparkFlexMotor) motors.get(port);
    }

    public static TalonFXSMotor createDefaultTalonFXS(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new TalonFXSMotor(config));
        }
        
        return (TalonFXSMotor) motors.get(port);
    }

    public static <M extends BaseMotor> boolean isOfMotorType(int port, Class<M> cls) {
        return motors.get(port) != null && motors.get(port).getClass().equals(cls);
    }
}
