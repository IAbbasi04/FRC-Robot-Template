package org.team8592.frc.robot.unittest;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class UnitTest extends Command {
    protected Timer timer = new Timer();
    
    public enum StatusType {
        kRunning("IS STILL RUNNING"),
        kSucceeded("PASSED"),
        kFailed("FAILED"),
        kGivenUp("ENDED WITH NO RESULT"),
        kError("ENCOUNTERED AN ERROR"),
        ;

        public String displayResult;
        private StatusType(String display) {
            this.displayResult = display;
        }
    }

    protected StatusType status = StatusType.kRunning;
    
    /**
     * Sets up everything needed to begin the testing sequence
     */
    public abstract Command initTest();

    public abstract Command createTest();

    /**
     * Did not achieve desired end result
     */
    public abstract boolean hasFailed();

    /**
     * Achieved desired end result
     */
    public abstract boolean hasSucceeded();

    /**
     * In case there is a specific event we want to cause a "give up" to pursue
     */
    public abstract boolean hasGivenUp();

    /**
     * Easily display exact what the unit test is checking for
     */
    public abstract String toString();

    /**
     * Code ran once after the completion of the unit test
     */
    public abstract Command onFinish();

    /**
     * Current test is considered finished if it has passed or failed the requirements
     */
    @Override
    public boolean isFinished() {
        return hasFailed() || hasSucceeded() || hasGivenUp();
    }

    /**
     * The current status of the unit test
     */
    public StatusType getStatus() {
        return this.status;
    }

    /**
     * Updates the status based on what the unit test end conditions return
     */
    public Command updateStatus() {
        return Commands.run(() -> {
            if (isFinished()) {
                if (hasFailed()) {
                    this.status = StatusType.kFailed;
                } else if (hasSucceeded()) {
                    this.status = StatusType.kSucceeded;
                } else if (hasGivenUp()) {
                    this.status = StatusType.kGivenUp;
                } else {
                    this.status = StatusType.kError;
                }
            } else {
                this.status = StatusType.kRunning;
            }
        }, new NewtonSubsystem[0]);
    }

    public Command updateTimer() {
        return Commands.run(() -> {
            timer.start();
        }, new NewtonSubsystem[0]);
    }
}