package lib.team8592.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartLogger {
    private String logFolder = "/CustomsLogs/";

    public SmartLogger(String name) {
        this.logFolder = logFolder + name + "/";
    }

    public void log(String key, double value) {
        Logger.recordOutput(logFolder + key, value);
    }

    public void log(String key, boolean value) {
        Logger.recordOutput(logFolder + key, value);
    }

    public void log(String key, StructSerializable value) {
        Logger.recordOutput(logFolder + key, value);
    }

    public void log(String key, String value) {
        Logger.recordOutput(logFolder + key, value);
    }

    public <T extends Enum<T>> void log(String key, T value) {
        Logger.recordOutput(logFolder + key, value);
    }

    public void logIf(String key, double valueIfTrue, double valueIfFalse, boolean condition) {
        double loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }

    public void logIf(String key, boolean valueIfTrue, boolean valueIfFalse, boolean condition) {
        boolean loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }

    public void logIf(String key, String valueIfTrue, String valueIfFalse, boolean condition) {
        String loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }

    public void logIf(String key, StructSerializable valueIfTrue, StructSerializable valueIfFalse, boolean condition) {
        StructSerializable loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }

    public <E extends Enum<E>> void logIf(String key, E valueIfTrue, E valueIfFalse, boolean condition) {
        E loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(logFolder + key, loggedValue); // Record to AdvantageKit logs
    }


    public static void addSendable(Sendable sendable) {
        SmartDashboard.putData(sendable);
    }

    public static void addSendable(String key, Sendable sendable) {
        SmartDashboard.putData(key, sendable);
    }
}