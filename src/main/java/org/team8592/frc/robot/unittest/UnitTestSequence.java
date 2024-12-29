package org.team8592.frc.robot.unittest;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class UnitTestSequence extends Command {
    private Command testCommand = null;

    /**
     * Pass all tests you want to run in into the UnitTestScheduler.
     * 
     * Treated like a queue of sorts since the tests run in order and not
     * simultaneously.
     */
    public List<UnitTest> testsToRun = new ArrayList<>();

    /**
     * If you want to assign a separate list than the default
     */
    public UnitTestSequence(List<UnitTest> tests) {
        this.testsToRun = tests;

        Command[] commandArray = new Command[tests.size()];
        for (int i = 0; i < tests.size(); i++) {
            UnitTest currentTest = tests.get(i);
            commandArray[i] = currentTest.initTest().andThen(
                tests.get(i).createTest().deadlineWith(
                    tests.get(i).updateStatus(),
                    tests.get(i).updateTimer()
                ).until(() -> currentTest.isFinished())
            ).andThen(tests.get(i).onFinish())
            .andThen(new WaitCommand(1.0));
        }

        if (commandArray.length != 0) {
            this.testCommand = new SequentialCommandGroup(commandArray)
                .until(() -> tests.get(tests.size()-1).isFinished());
        } else {
            this.testCommand = Commands.none();
        }
    }

    @Override
    public void initialize() {
        testCommand.initialize();
    }

    @Override
    public void execute() {
        testCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return testCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        testCommand.end(interrupted);
        if (interrupted) return; // Do nothing if test plan was interrupted

        int index = 1;
        for (UnitTest test : testsToRun) {
            System.out.println("\n\n=================================================================================");
            String displayedResult = String.format("Test [%s]: ", index) + test.toString();
            String displayedStatus = test.getStatus().displayResult;
            String displayColor = "";
            switch (test.getStatus()) {
                case kFailed:
                    displayColor = "\u001B[31m"; // Red
                    break;
                case kSucceeded:
                    displayColor = "\u001B[32m"; // Green
                    break;
                case kGivenUp:
                    displayColor = "\u001B[35m"; // Purple
                    break;
                default:
                    displayColor = "\u001B[33m"; // Yellow
                    break;
            }

            SmartDashboard.putString("Unit Tests/" + displayedResult, displayedStatus);
            
            displayedResult += "... " + displayColor + displayedStatus + "\u001B[0m";
            System.out.println(displayedResult);
            System.out.println("=================================================================================\n\n");

            index++;
        }
    }
}