package org.team8592.frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import static org.team8592.frc.robot.Constants.CONFIG.*;

/**
 * Credit to 6328 Mechanical Advantage for the original version of this class.
 */
public class RobotSelector {
    public static RobotType getRobot() {
        return (RobotBase.isReal() || IGNORE_SIMBOT) ? ROBOT_TYPE : RobotType.SIM_BOT;
    }

    public static enum RobotType {
        COMP_BOT,
        PRAC_BOT,
        SIM_BOT
    }
}