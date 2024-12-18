package com.frc.robot.unittest;

public abstract class UnitTest {
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
    public abstract void initialize();

    /**
     * Runs the testing sequence
     */
    public abstract void run();

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
    public abstract void shutdown();

    /**
     * Current test is considered finished if it has passed or failed the requirements
     */
    public boolean hasFinished() {
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
    public void updateStatus() {
        if (hasFinished()) {
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
    }
}