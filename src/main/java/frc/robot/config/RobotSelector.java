package frc.robot.config;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Utility class that determines the type of robot being used based on the environment
 * 
 * Credit to 6328 Mechanical Advantage for the original version of this class
 */
public class RobotSelector {
    public static RobotType getRobot() {
        return (RobotBase.isReal() || RobotConfig.IGNORE_SIMBOT) ? RobotConfig.ROBOT_TYPE : RobotType.SIM_BOT;
    }

    public static enum RobotType {
        // Add all usable robots here
        COMP_BOT,
        SIM_BOT
    }
}