package org.team8592.frc.robot.commands.largecommands;

import org.team8592.frc.robot.Constants.SWERVE;
import org.team8592.frc.robot.subsystems.SwerveSubsystem;
import org.team8592.lib.pathing.newtonpath.NewtonTrajectory;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.geometry.*;

public class NewtonFollowerCommand extends LargeCommand {
    private SwerveControllerCommand pathFollowerCommand;

    private ProfiledPIDController turnController;
    private HolonomicDriveController drivePID;
    private PIDController xController;
    private PIDController yController;

    public NewtonFollowerCommand(SwerveSubsystem swerve, NewtonTrajectory trajectory) {
        super(swerve);

        this.xController = new PIDController(
            SWERVE.PATH_FOLLOW_TRANSLATE_kP,
            SWERVE.PATH_FOLLOW_TRANSLATE_kI,
            SWERVE.PATH_FOLLOW_TRANSLATE_kD
        );

        this.xController.setTolerance(
            SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
        );

        this.yController = new PIDController(
            SWERVE.PATH_FOLLOW_TRANSLATE_kP,
            SWERVE.PATH_FOLLOW_TRANSLATE_kI,
            SWERVE.PATH_FOLLOW_TRANSLATE_kD
        );

        this.yController.setTolerance(
            SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
        );

        this.turnController = new ProfiledPIDController(
            SWERVE.PATH_FOLLOW_ROTATE_kP,
            SWERVE.PATH_FOLLOW_ROTATE_kI,
            SWERVE.PATH_FOLLOW_ROTATE_kD,
            new Constraints(
                SWERVE.PATH_FOLLOW_ROTATE_MAX_VELOCITY,
                SWERVE.PATH_FOLLOW_ROTATE_MAX_ACCELLERATION
            )
        );

        this.turnController.setTolerance(
            SWERVE.PATH_FOLLOW_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_VELOCITY_TOLERANCE
        );
        
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);

        this.drivePID = new HolonomicDriveController(xController, yController, turnController);
        this.drivePID.setTolerance(
            new Pose2d(
                new Translation2d(xController.getPositionTolerance(), yController.getPositionTolerance()),
                new Rotation2d(turnController.getPositionTolerance())
            )
        );

        this.pathFollowerCommand = new SwerveControllerCommand(
            trajectory.getTrajectory(), 
            swerve::getCurrentPosition,
            swerve.getKinematics(), 
            this.xController,
            this.yController,
            this.turnController, 
            () -> trajectory.getEndRotation(),
            swerve::setModuleStates, 
            swerve
        );
    }

    @Override
    public void initialize() {
        this.pathFollowerCommand.initialize();
    }

    @Override
    public void execute() {
        this.pathFollowerCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        this.pathFollowerCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pathFollowerCommand.isFinished();
    }
}