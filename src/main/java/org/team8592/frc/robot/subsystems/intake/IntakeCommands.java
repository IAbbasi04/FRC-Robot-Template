package org.team8592.frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommands {
    private IntakeSubsystem intake;
    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public Command setIntakePowerCommand(double power) {
        return intake.run(() -> intake.setPower(power));
    }

    public Command setIntakeVelocityCommand(double velocityRPM) {
        return intake.run(() -> intake.setVelocity(velocityRPM));
    }
}