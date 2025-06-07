package frc.robot.autonomous;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.SwerveConstants;
import lib.MatchMode;
import lib.commands.MultiComposableCommand;

public class AutoLoader {
    private SubsystemManager manager;

    private SendableChooser<Class<?>> autoChooser = new SendableChooser<>();
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous Config");
    private GenericEntry startDelayEntry;
    
    // private Class<?>[] autos = new Class<?>[]{
    //     TestAuto.class
    // };

    public AutoLoader(SubsystemManager manager) {
        this.manager = manager;

        try {
            AutoBuilder.configure(
                manager.swerve::getCurrentPosition, 
                (pose) -> {manager.swerve.resetPose(pose);}, 
                manager.swerve::getWheelSpeeds, 
                (speeds) -> {manager.swerve.driveFieldOriented(speeds);},
                new PPHolonomicDriveController(
                    SwerveConstants.PATH_FOLLOW_TRANSLATE_GAINS.toPIDConstants(), 
                    SwerveConstants.PATH_FOLLOW_ROTATE_GAINS.toPIDConstants()
                ),
                RobotConfig.fromGUISettings(),
                () -> 
                    DriverStation.getAlliance().isPresent() && 
                    DriverStation.getAlliance().get() == Alliance.Red,
                manager.swerve);
        } catch (Exception e) {
            System.out.println("GUI Settings not properly configured for PathPlanner");
        }

        initAutoSelector();
    }

    private void initAutoSelector() {
        List<BaseAuto> autos = new ArrayList<>();
        String packageName = "/src/main/java/frc/robot/autonomous/autos";
        File autosDir = new File(System.getProperty("user.dir") + packageName);

        this.autoChooser.setDefaultOption("DEFAULT - DO NOTHING", BaseAuto.getDefaultAuto().getClass());

        this.autoTab.add("Auto Selector", autoChooser)
            .withPosition(3, 3)
            .withSize(4, 2);
        
        this.startDelayEntry = autoTab.add("Start Delay", 0d)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();

        if(autosDir.isDirectory()) {
            File[] files = autosDir.listFiles((dir, name) -> name.endsWith(".java"));
            ClassLoader classLoader = getClass().getClassLoader();

            try {
                if (files != null) {
                    for (File file : files) {
                        String className = file.getName().replace(".java", "");
                        String fullClassName = "frc.robot.autonomous.autos." + className;

                        Class<?> auto = classLoader.loadClass(fullClassName);

                        if (BaseAuto.class.isAssignableFrom(auto)) {
                            BaseAuto baseAuto = (BaseAuto) auto.getDeclaredConstructor().newInstance();
                            autos.add(baseAuto);
                            autoChooser.addOption(baseAuto.getName(), auto);
                        }
                    }
                }
            } catch (Exception e) {}
        }
    }

    public Command loadSelectedAuto() {
        return manager.onModeInitCommand(MatchMode.AUTONOMOUS)
        .andThen(
            manager.swerve.resetHeading(),
            manager.swerve.resetAlliancePose(getSelectedAuto().getInitialPose()),
            getStartDelay(), 
            new MultiComposableCommand(getSelectedAuto())
        );
    }

    private BaseAuto getSelectedAuto() {
        try {
            return (BaseAuto) autoChooser.getSelected().getDeclaredConstructor().newInstance();
        } catch (Exception e) {
            return BaseAuto.getDefaultAuto();
        }
    }

    private Command getStartDelay() {
        return new WaitCommand(startDelayEntry.getDouble(0d));
    }
}