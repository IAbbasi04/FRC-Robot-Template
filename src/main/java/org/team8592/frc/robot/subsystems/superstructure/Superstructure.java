package org.team8592.frc.robot.subsystems.superstructure;

public class Superstructure {
    // TODO - Put mech2d here
    public enum States {
        ;

        public double wristDegrees, shoulderDegrees, elevatorInches;
        private States(double wrist, double shoulder, double elevator){
            this.wristDegrees = wrist;
            this.shoulderDegrees = shoulder;
            this.elevatorInches = elevator;
        }
    }
}