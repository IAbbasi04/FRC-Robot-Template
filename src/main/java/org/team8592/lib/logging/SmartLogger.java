package org.team8592.lib.logging;

import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import org.team8592.lib.logging.LogUtils.WidgetProfile;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SmartLogger {
    private String tableName = "";
    private ShuffleboardTab shuffleboardTab;
    // private Dictionary<String, GenericEntry> cards;

    // private ArrayList<LoggerEntry> _cards = new ArrayList<>();

    private LoggerDictionary dictionary = new LoggerDictionary();

    private boolean logToShuffleboard = true;
    private boolean initialized = false;

    public SmartLogger(String name, boolean logToShuffleboard) {
        this.tableName = name;
        this.logToShuffleboard = logToShuffleboard;
        // this.cards = new Hashtable<>();
    }

    public SmartLogger(String name) {
        this(name, true);
    }

    public ShuffleboardTab getShuffleboardTab() {
        return this.shuffleboardTab;
    }

    public List<GenericEntry> getAllLoggedEntries() {
        return dictionary.getAllGenericEntries();
    }

    public SmartLogger initialize() {
        if (logToShuffleboard && !initialized()) {
            this.shuffleboardTab = Shuffleboard.getTab(tableName);
            this.initialized = true;
        }

        return this;
    }

    public boolean initialized() {
        return initialized;
    }

    public SmartLogger enable(boolean enable) {
        this.logToShuffleboard = enable;
        return this;
    }

    public SmartLogger enable() {
        return this.enable(true);
    }

    public SmartLogger disable() {
        return this.enable(false);
    }

    public SmartLogger addSendable(Sendable sendable) {
        this.shuffleboardTab.add(sendable);
        return this;
    }

    public GenericEntry getEntry(String key) {
        return this.dictionary.getGenericEntry(key);
    }

    public void log(String key, double value) {
        Logger.recordOutput(tableName + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, value).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setDouble(value);
        }
    }

    public void log(String key, String value) {
        Logger.recordOutput(tableName + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, value).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setString(value);
        }
    }

    public void log(String key, boolean value) {
        Logger.recordOutput(tableName + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, value).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setBoolean(value);
        }
    }

    public <E extends Enum<E>> void log(String key, E value) {
        Logger.recordOutput(tableName + "/" + key, value.name()); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, value.name()).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setString(value.name());
        }
    }

    public void log(String key, StructSerializable value) {
        Logger.recordOutput(tableName + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, value.toString()).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setString(value.toString());
        }
    }

    public void logReceiver(String key, boolean defaultValue, Consumer<Boolean> consumer) {
        if (!dictionary.contains(key)) { // Do not log if already logged
            this.log(key, defaultValue);
        }

        if (!dictionary.getLoggerEntry(key).override()) {
            consumer.accept(dictionary.getGenericEntry(key).getBoolean(defaultValue));
        }
    }
    
    public void overrideReceiver(String key, boolean override) {
        if (dictionary.contains(key)) {
            dictionary.getLoggerEntry(key).override(override);
        }
    }

    public <T> SendableChooser<T> createSendableChooserWithDefault(T defaultChoice, List<T> choices) {
        SendableChooser<T> chooser = new SendableChooser<>();
        chooser.setDefaultOption(defaultChoice.toString(), defaultChoice);
        for (T choice : choices) {
            chooser.addOption(choice.toString(), choice);
        }
        return chooser;
    }

    public void addSendable(Sendable sendable, WidgetProfile constants) {
        this.shuffleboardTab.add(sendable).withPosition(constants.column, constants.row);
    }

    public void logIf(String key, double valueIfTrue, double valueIfFalse, boolean condition) {
        double loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        
        Logger.recordOutput(tableName + key, loggedValue); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, loggedValue).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setDouble(loggedValue);
        }
    }

    public void logIf(String key, String valueIfTrue, String valueIfFalse, boolean condition) {
        String loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        Logger.recordOutput(tableName + key, loggedValue); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, loggedValue).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setString(loggedValue);
        }
    }

    public void logIf(String key, boolean valueIfTrue, boolean valueIfFalse, boolean condition) {
        boolean loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }
        Logger.recordOutput(tableName + key, loggedValue); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, loggedValue).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setBoolean(loggedValue);
        }
    }

    public <E extends Enum<E>> void logIf(String key, E valueIfTrue, E valueIfFalse, boolean condition) {
        E loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }

        Logger.recordOutput(tableName + key, loggedValue.name()); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, loggedValue.name()).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setString(loggedValue.name());
        }
    }

    public void logIf(String key, StructSerializable valueIfTrue, StructSerializable valueIfFalse, boolean condition) {
        StructSerializable loggedValue = valueIfFalse;
        if (condition) {
            loggedValue = valueIfTrue;
        }

        Logger.recordOutput(tableName + key, loggedValue); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (!dictionary.contains(key)) { // Card doesn't exist yet
            dictionary.addEntry(key, new LoggerEntry(key, shuffleboardTab.add(key, loggedValue).getEntry()));
        } else { // Card already exists
            dictionary.getGenericEntry(key).setString(loggedValue.toString());
        }
    }
}