package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Suppliers;
import frc.robot.commands.MultiComposableCommand;
import frc.robot.subsystems.SubsystemManager;
import lib.team1731.MatchMode;

public class AutoLoader {
    private SubsystemManager manager;

    private SendableChooser<BaseAuto> autoChooser = new SendableChooser<>();
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous Config");
    private GenericEntry startDelayEntry;
    
    private BaseAuto[] autos = new BaseAuto[]{
        
    };

    public AutoLoader(SubsystemManager manager) {
        this.manager = manager;

        this.autoChooser.setDefaultOption("DEFAULT - DO NOTHING", BaseAuto.getDefaultAuto());
        for (BaseAuto auto : autos) {
            try {
                autoChooser.addOption(auto.getClass().getSimpleName(), auto);
            } catch (Exception e) {
                System.err.println("Failed to load auto: " + auto.getClass().getSimpleName());
            }
        }

        this.autoTab.add("Auto Selector", autoChooser)
            .withPosition(3, 3)
            .withSize(4, 2);
        
        this.startDelayEntry = autoTab.add("Start Delay", 0d)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();

        try {
            AutoBuilder.configure(
                manager.swerve::getCurrentPosition, 
                (pose) -> {manager.swerve.resetPose(pose);}, 
                manager.swerve::getWheelSpeeds, 
                (speeds) -> {manager.swerve.driveFieldOriented(speeds);},
                new PPHolonomicDriveController(
                    Constants.SWERVE.PATH_FOLLOW_TRANSLATE_GAINS.toPIDConstants(), 
                    Constants.SWERVE.PATH_FOLLOW_ROTATE_GAINS.toPIDConstants()
                ),
                RobotConfig.fromGUISettings(),
                Suppliers.IS_RED_ALLIANCE, 
                manager.swerve
            );
        } catch (Exception e) {
            System.out.println("GUI Settings not properly configured for PathPlanner");
        }
    }

    private Command getStartDelay() {
        return new WaitCommand(startDelayEntry.getDouble(0d));
    }

    public Command loadSelectedAuto() {
        return manager.onModeInitCommand(MatchMode.AUTONOMOUS)
        .andThen(
            manager.swerve.resetHeading(),
            manager.swerve.resetAlliancePose(autoChooser.getSelected().getInitialPose()),
            getStartDelay(), 
            new MultiComposableCommand(autoChooser.getSelected())
        );
    }
}