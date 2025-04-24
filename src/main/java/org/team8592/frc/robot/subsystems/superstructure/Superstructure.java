package org.team8592.frc.robot.subsystems.superstructure;

import org.team8592.frc.robot.Suppliers;

public class Superstructure {
    // TODO - Put mech2d here
    public enum States {
        ;

        public double wristDegrees, shoulderDegrees, elevatorInches;
        private States(double compWrist, double compShoulder, double compElevator, double pracWrist, double pracShoulder, double pracElevator){
            // if (Suppliers.)
            this.wristDegrees = compWrist;
            this.shoulderDegrees = compShoulder;
            this.elevatorInches = compElevator;


        }
    }
}