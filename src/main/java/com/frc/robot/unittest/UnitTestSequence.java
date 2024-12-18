package com.frc.robot.unittest;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.frc.robot.unittest.UnitTest.StatusType;

import com.frc.robot.Suppliers;

public class UnitTestSequence extends Command {
    private Timer sequenceTimer = new Timer();
    private UnitTest currentTest;
    
    private List<StatusType> testResults = new ArrayList<>(); // Whether each test passed or failed
    private List<UnitTest> ranTests = new ArrayList<>(); // Tests that have already finished

    private int currentIndex = 0;

    /**
     * Pass all tests you want to run in Robot.testPeriodic().
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
    }

    @Override
    public void initialize() {
        sequenceTimer.reset();
        sequenceTimer.start();
        currentTest = testsToRun.get(0);
        currentTest.initialize();
    }

    @Override
    public void execute() {
        if (!currentTest.equals(null)) { // If there are tests to run
            currentTest.run(); // Runs through the current test plan
            currentTest.updateStatus(); // Updates the ending status of the current unit test
            if (currentTest.hasFinished()) { // Test has completed
                currentTest.shutdown(); // Run the shutdown() of the currently running command
                testResults.add(currentTest.getStatus()); // Records the ending status of the current test
                ranTests.add(currentTest);
                currentIndex++;

                if (currentIndex == testsToRun.size()) { // No more tests left
                    currentTest = null;
                } else { // Move on to the next test
                    currentTest = testsToRun.get(currentIndex);
                    currentTest.initialize();
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return currentTest == null;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) return; // Do nothing if test plan was interrupted

        if (ranTests.size() != testResults.size()) {
            System.out.println("\n\n===========================================================");
            System.out.println("\u001B[31mError: Number of Ran Tests Does Not Match Number of Results\u001B[0m");
            System.out.println("===========================================================\n\n");
            return;
        }

        for (int i = 0; i < ranTests.size(); i++) {
            System.out.println("\n\n===========================================================");
            String displayedResult = ranTests.get(i).toString();
            String displayedStatus = ranTests.get(i).getStatus().displayResult;
            String displayColor = "";
            switch (ranTests.get(i).getStatus()) {
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

            if (Suppliers.robotRunningOnRed.getAsBoolean()) {
                SmartDashboard.putString("Unit Tests/" + displayedResult, displayedStatus);
            }
            
            displayedResult += "... " + displayColor + displayedStatus + "\u001B[0m";
            System.out.println(displayedResult);
            System.out.println("===========================================================\n\n");
        }
    }
}