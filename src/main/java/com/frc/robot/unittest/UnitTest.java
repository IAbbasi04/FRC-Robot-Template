package com.frc.robot.unittest;

public abstract class UnitTest {
    protected String testName = "";

    public static UnitTest none() {
        return new UnitTest() {
            @Override
            public void initialize() {}

            @Override
            public void update() {}

            @Override
            public boolean isFinished() { return true; }

            @Override
            public void shutdown() {}

            @Override
            public String getTestName() {
                return "No Test";
            }
        };
    }

    public abstract void initialize();
    
    public abstract void update();

    public abstract boolean isFinished();

    public abstract void shutdown();

    public abstract String getTestName();

    @Override
    public String toString() {
        this.testName = getTestName();
        return getTestName();
    }
}