package lib.team8592.hardware.motor;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

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