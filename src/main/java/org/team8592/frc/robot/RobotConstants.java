package org.team8592.frc.robot;

import java.util.Map;

import org.team6328.lib.Alert;
import org.team6328.lib.Alert.AlertType;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Credit to 6328 Mechanical Advantage for the original version of this class.
 */
public class RobotConstants {
    private static final RobotType robot = RobotType.COMP_BOT;
    public static final boolean tuningMode = false;

    public static boolean invalidRobotAlertSent = false;

    public static RobotType getRobot() {
        if (!disableHAL && !RobotBase.isReal()) {
            if (robot == RobotType.SIM_BOT) { // Invalid robot selected
                if (!invalidRobotAlertSent) {
                    new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
                        .set(true);
                    invalidRobotAlertSent = true;
                }
                return RobotType.COMP_BOT;
            } else {
                return robot;
            }
        } else {
            if (RobotBase.isSimulation()) return RobotType.SIM_BOT;
            return robot;
        }
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case COMP_BOT:
            case PRAC_BOT:
            return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

            case SIM_BOT:
            return Mode.SIM;

            default:
            return Mode.REAL;
        }
    }

    public static final Map<RobotType, String> logFolders =
        Map.of(RobotType.COMP_BOT, "/media/sda2/");

    public static enum RobotType {
        COMP_BOT,
        PRAC_BOT,
        SIM_BOT
    }

    public static enum Mode {
        REAL,
        REPLAY,
        SIM
    }

    // Function to disable HAL interaction when running without native libs
    public static boolean disableHAL = false;

    public static void disableHAL() {
        disableHAL = true;
    }

    /** Checks whether the robot the correct robot is selected when deploying. */
    public static void main(String... args) {
        if (robot == RobotType.SIM_BOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
            System.exit(1);
        }
    }
}
