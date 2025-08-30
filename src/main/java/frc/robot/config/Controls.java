package frc.robot.config;


import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.*;
import lib.logging.SmartLogger;

/**
 * Class that handles driver and operator inputs
 */
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

    private static SmartLogger logger = new SmartLogger("Controls");

    protected static DoubleSupplier driveTranslateX = () -> -driverController.getLeftX();
    protected static DoubleSupplier driveTranslateY = () -> -driverController.getLeftY();
    protected static DoubleSupplier driveRotate = () -> -driverController.getRightX();

    protected static Trigger slowMode = driverController.rightBumper();
    // protected static Trigger robotRelative = driverController.leftBumper();
    protected static Trigger zeroGryoscope = driverController.back();

    protected static Trigger shift = operatorController.start();

    protected static Trigger snapNorth = driverController.pov(0);
    protected static Trigger snapSouth = driverController.pov(180);
    protected static Trigger snapWest = driverController.pov(270);
    protected static Trigger snapEast = driverController.pov(90);

    protected static Trigger intake = driverController.leftTrigger();
    protected static Trigger score = driverController.rightTrigger();

    protected static Trigger stow = driverController.a();

    protected static Trigger primeGroundIntake = operatorController.leftTrigger();
    protected static Trigger primeChuteIntake = operatorController.leftBumper();
    protected static Trigger primeShelfIntake = operatorController.rightBumper();

    protected static Trigger prime = operatorController.b();
    protected static Trigger primeStow = driverController.leftBumper();
    protected static Trigger primeLow = operatorController.a();
    protected static Trigger primeMid = operatorController.x();
    protected static Trigger primeHigh = operatorController.y();

    protected static Trigger setCubeMode = operatorController.pov(90).and(() -> !shift.getAsBoolean());
    protected static Trigger setConeMode = operatorController.pov(270).and(() -> !shift.getAsBoolean());
    protected static Trigger flashPartyMode = operatorController.pov(0).and(() -> !shift.getAsBoolean());

    protected static Trigger trimWristUp = operatorController.pov(0).and(() -> shift.getAsBoolean());
    protected static Trigger trimWristDown = operatorController.pov(180).and(() -> shift.getAsBoolean());

    protected static Trigger throwPiece = operatorController.rightTrigger();

    /**
     * Change the variables in the Controls class to match the specified
     * control set. Note that this doesn't edit or remove bindings.
     *
     * @param set the control set to apply
     */
    protected static void applyControlSet(ControlSets set){
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

    /**
     * Logs the current state of all controls
     */
    public static void logControls() {
        for (Field field : Controls.class.getDeclaredFields()) {
            try {
                logger.log(field.getName(), ((Trigger)field.get(null)).getAsBoolean());
            } catch (Exception e) {}
        }
    }

    /**
     * Returns the driver controller
     */
    protected static CommandXboxController getDriver() {
        return driverController;
    }

    /**
     * Returns the operator controller
     */
    protected static CommandXboxController getOperator() {
        return operatorController;
    }
}