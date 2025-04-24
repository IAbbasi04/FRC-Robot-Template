package org.team8592.frc.robot;

import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.*;
import org.team8592.lib.logging.SmartLogger;

public final class Controls {
    private static final CommandXboxController driverController = new CommandXboxController(0);
    private static final CommandXboxController operatorController = new CommandXboxController(1);
    private static final CommandGenericHID operatorGamePad = new CommandGenericHID(2);

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
    protected static Trigger robotRelative = driverController.leftBumper();
    protected static Trigger zeroGryoscope = driverController.back();

    protected static Trigger snapNorth = driverController.pov(0);
    protected static Trigger snapSouth = driverController.pov(180);
    protected static Trigger snapWest = driverController.pov(270);
    protected static Trigger snapEast = driverController.pov(90);

    protected static Trigger score = driverController.rightTrigger();

    protected static Trigger intake = driverController.leftTrigger();
    // protected static Trigger groundAlgaeIntake = driverController.leftTrigger().and(Suppliers.IS_ALGAE_MODE);

    protected static Trigger stow = driverController.a();//.or(driverController.leftTrigger().and(Suppliers.IS_CORAL_MODE));
    // protected static Trigger algaeStow = driverController.a().and(Suppliers.IS_ALGAE_MODE);

    protected static Trigger goToTargetPosition = driverController.x();

    protected static Trigger deepClimbInitialize = driverController.start().and(operatorController.start().or(driverController.b()));

    protected static Trigger setCoralMode = operatorController.pov(90);
    protected static Trigger setAlgaeMode = operatorController.pov(270);
    protected static Trigger setDeepClimbMode = operatorController.pov(180);

    protected static Trigger autoLockLeftBranch = driverController.leftBumper().or(
        operatorGamePad.button(1)
            .or(operatorGamePad.button(4))
                .or(operatorGamePad.button(6))
                    .or(operatorGamePad.button(5))
    ).and(Suppliers.IS_CORAL_MODE);

    protected static Trigger autoLockRightBranch = driverController.rightBumper().or(
        operatorGamePad.button(2)
            .or(operatorGamePad.button(3))
                .or(operatorGamePad.button(8))
                    .or(operatorGamePad.button(7))
    ).and(Suppliers.IS_CORAL_MODE);

    protected static Trigger primeL1 = (operatorController.a().or(operatorGamePad.button(4).or(operatorGamePad.button(3)))).and(Suppliers.IS_CORAL_MODE);
    protected static Trigger primeL2 = (operatorController.x().or(operatorGamePad.button(1).or(operatorGamePad.button(2)))).and(Suppliers.IS_CORAL_MODE);
    protected static Trigger primeL3 = (operatorController.b().or(operatorGamePad.button(6).or(operatorGamePad.button(8)))).and(Suppliers.IS_CORAL_MODE);
    protected static Trigger primeL4 = (operatorController.y().or(operatorGamePad.button(5).or(operatorGamePad.button(7)))).and(Suppliers.IS_CORAL_MODE);

    protected static Trigger primeProcessor = (operatorController.a().or(operatorGamePad.button(4))).and(Suppliers.IS_ALGAE_MODE);
    protected static Trigger primeL2Algae = (operatorController.x().or(operatorGamePad.button(1))).and(Suppliers.IS_ALGAE_MODE);
    protected static Trigger primeL3Algae = (operatorController.b().or(operatorGamePad.button(6))).and(Suppliers.IS_ALGAE_MODE);
    protected static Trigger primeNet = (operatorController.y().or(operatorGamePad.button(5))).and(Suppliers.IS_ALGAE_MODE);

    protected static Trigger manualDeepClimbUp = driverController.pov(0).and(Suppliers.IS_CLIMB_MODE);
    protected static Trigger manualDeepClimbDown = driverController.pov(180).and(Suppliers.IS_CLIMB_MODE);

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

    public static void logControls() {
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

    protected static CommandGenericHID getOperatorGamePad() {
        return operatorGamePad;
    }
}