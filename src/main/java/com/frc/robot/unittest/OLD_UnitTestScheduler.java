package com.frc.robot.unittest;

@Deprecated
public class OLD_UnitTestScheduler {
    // private SendableChooser<UnitTest> testChooser;
    // private SubsystemManager manager;
    // private Command testCommand = null;

    // public UnitTestScheduler(SubsystemManager manager) {
    //     this.manager = manager;
    //     this.testChooser = LogUtils.createSendableChooserWithDefault(
    //         UnitTest.none(),
    //         List.of(
    //             // Add all unit tests here
    //             new DriveForTwoSecondsTest(manager)
    //         )
    //     );

    //     LogUtils.addSendable(
    //         "Unit Tests", 
    //         testChooser
    //     );
    // }

    // public Command getSelectedTest() {
    //     UnitTest test = testChooser.getSelected();

    //     /**
    //      * To run a unit test we run it like a simple LongCommand.java essentially
    //      * 
    //      * test.initialize() -> will run once at the start of the test to 
    //      *      set up anything additional
    //      * 
    //      * test.update() -> will run constantly through the duration of the
    //      *      test until the end condition is met
    //      * 
    //      * test.isFinished() -> the ending condition for this unit test; will
    //      *      still end after being disabled and reset regardless
    //      * 
    //      * test.shutdown() -> the snippet of code that is ran after the completion
    //      *      of the test to either reset or stop the robot
    //      */
    //     this.testCommand = 
    //         Commands.runOnce(
    //             () -> {
    //                 test.initialize();
    //             }, 
    //             manager.getAllSubsystemsAsArray()
    //         ).andThen(
    //             Commands.runEnd(
    //                 () -> {
    //                     test.update();
    //                 }, 
    //                 () -> {
    //                     test.shutdown();
    //                 }, 
    //                 manager.getAllSubsystemsAsArray()
    //             ).until(() -> test.isFinished())
    //         ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    //     return this.testCommand;
    // }
}