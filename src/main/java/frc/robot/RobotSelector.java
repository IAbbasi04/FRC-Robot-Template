package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Credit to 6328 Mechanical Advantage for the original version of this class.
 */
public class RobotSelector {
    public static RobotType getRobot() {
        return (RobotBase.isReal() || RobotConfig.IGNORE_SIMBOT) ? RobotConfig.ROBOT_TYPE : RobotType.SIM_BOT;
    }

    public static enum RobotType {
        COMP_BOT,
        DEV_BOT,
        SIM_BOT
    }
}