package org.team8592.frc.robot;


import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team8592.lib.logging.SmartLogger;

public final class Controls {
    private static final CommandXboxController driverController = new CommandXboxController(0);
    private static final CommandXboxController operatorController = new CommandXboxController(1);

    /**
     * Enum for the different sets of controls (different drivers,
     * different DS configurations, etc)
     */
    protected enum ControlSets{
        SINGLE_DRIVER,
        DUAL_DRIVER
    }

    private static SmartLogger logger;

    private static boolean logToShuffleboard = false;
    private static boolean loggingEnabled = false;

    protected static DoubleSupplier driveTranslateX = () -> 0;
    protected static DoubleSupplier driveTranslateY = () -> 0;
    protected static DoubleSupplier driveRotate = () -> 0;

    protected static Trigger slowMode = new Trigger(() -> false);
    protected static Trigger robotRelative = new Trigger(() -> false);
    protected static Trigger zeroGryoscope = new Trigger(() -> false);

    protected static Trigger snapNorth = new Trigger(() -> false);
    protected static Trigger snapSouth = new Trigger(() -> false);
    protected static Trigger snapWest = new Trigger(() -> false);
    protected static Trigger snapEast = new Trigger(() -> false);

    /**
     * Sets the controls for the drivebase movement
     */
    private static void applyDrivetrainControls() {
        driveTranslateX = () -> -driverController.getLeftX();
        driveTranslateY = () -> -driverController.getLeftY();
        driveRotate = () -> -driverController.getRightX();

        slowMode = driverController.rightBumper();
        robotRelative = driverController.leftBumper();
        zeroGryoscope = driverController.back();

        snapNorth = driverController.pov(0);
        snapSouth = driverController.pov(180);
        snapWest = driverController.pov(270);
        snapEast = driverController.pov(90);
    }

    /**
     * Change the variables in the Controls class to match the specified
     * control set. Note that this doesn't edit or remove bindings.
     *
     * @param set the control set to apply
     */
    protected static void applyControlSet(ControlSets set){
        Controls.applyDrivetrainControls();

        // Add controls that do not rely on control set below

        // Use the controls set if we have differentiating inputs for a certain control
        // i.e. if single driver intaking is driver left trigger whereas 
        //        dual driver has operator left trigger
        switch(set) {
            case SINGLE_DRIVER:

                break;
            default: case DUAL_DRIVER:

                break;
        }
        
    }

    public static void initializeShuffleboardLogs(boolean logToShuffleboard) {
        Controls.logToShuffleboard = logToShuffleboard;
        if (!logToShuffleboard) return;
        
        Controls.loggingEnabled = true;
        Controls.logger = new SmartLogger("Controls");
        Controls.logger.enable();
    }

    public static void logControlsToShuffleboard() {
        if (!Controls.logToShuffleboard) return; // Don't log if we don't want to log
        
        if (!Controls.loggingEnabled) { // If not already enabled, enable it
            initializeShuffleboardLogs(true);
        }

        for (Field field : Controls.class.getDeclaredFields()) {
            try {
                logger.log(field.getName(), ((Trigger)field.get(null)).getAsBoolean());
            } catch (Exception e) {}
        }
    }

    protected static CommandXboxController getDriver() {
        return driverController;
    }

    protected static CommandXboxController getOperator() {
        return operatorController;
    }
}