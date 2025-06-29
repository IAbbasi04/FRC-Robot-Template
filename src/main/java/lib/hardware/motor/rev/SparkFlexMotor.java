package lib.hardware.motor.rev;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import lib.hardware.motor.PortConfig;

/**
 * Wrapper class for motors that use the Spark Flex motor controller
 */
public class SparkFlexMotor extends SparkBaseMotor<SparkFlex, SparkFlexConfig> {
    public SparkFlexMotor(PortConfig config) {
        super(
            config.kBus,
            new SparkFlex(config.kPort, MotorType.kBrushless), 
            new SparkFlexConfig(),
            config.kInverted
        );
    }
}