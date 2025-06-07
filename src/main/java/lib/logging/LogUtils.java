package lib.logging;

import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class LogUtils {
    public static class LogConstants {
        public final String game, year, robot, team;
        public LogConstants(String game, String year, String robot, String team) {
            this.game = game;
            this.year = year;
            this.robot = robot;
            this.team = team;
        }
    }

    public static class WidgetProfile {
        public int height = 0;
        public int width = 0;
        public int column = 0;
        public int row = 0;

        public WidgetProfile withPosition(int column, int row) {
            this.column = column;
            this.row = row;
            return this;
        }

        public WidgetProfile withSize(int width, int height) {
            this.width = width;
            this.height = height;
            return this;
        }
    }

    public static void logToSmartDashboard(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public static void logToSmartDashboard(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    public static void logToSmartDashboard(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public static void addSendable(String sendableName, Sendable sendable) {
        SmartDashboard.putData(sendableName, sendable);
    }

    public static void addSendable(String tabName, String sendableName, Sendable sendable) {
        Shuffleboard.getTab(tabName).add(sendableName, sendable);
    }

    public static void addSendable(String tabName, String sendableName, Sendable sendable, WidgetProfile constants) {
        Shuffleboard.getTab(tabName).add(sendableName, sendable).withPosition(constants.column, constants.row);
    }

    public static <T> SendableChooser<T> createSendableChooserWithDefault(T defaultChoice, List<T> choices) {
        SendableChooser<T> chooser = new SendableChooser<>();
        chooser.setDefaultOption(defaultChoice.toString(), defaultChoice);
        for (T choice : choices) {
            chooser.addOption(choice.toString(), choice);
        }
        return chooser;
    }
}