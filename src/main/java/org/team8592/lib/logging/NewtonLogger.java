package org.team8592.lib.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewtonLogger {
    private String logFolder = "/CustomsLogs/";

    public NewtonLogger(String name) {
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

    public static void addSendable(Sendable sendable) {
        SmartDashboard.putData(sendable);
    }

    public static void addSendable(String key, Sendable sendable) {
        SmartDashboard.putData(key, sendable);
    }
}