package frc.robot.autonomous;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SubsystemManager;

import lib.MatchMode;
import lib.commands.MultiComposableCommand;

public class AutoLoader {
    private SubsystemManager manager;

    private SendableChooser<BaseAuto> autoChooser = new SendableChooser<>();
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous Config");
    private GenericEntry startDelayEntry;

    public AutoLoader(SubsystemManager manager) {
        this.manager = manager;
        initAutoSelector(manager);
    }

    private void initAutoSelector(SubsystemManager manager) {
        String packageName = "/src/main/java/frc/robot/autonomous/autos";
        File autosDir = new File(System.getProperty("user.dir") + packageName);

        this.autoChooser.setDefaultOption("DEFAULT - DO NOTHING", BaseAuto.getDefaultAuto());

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

            if (files != null) {
                for (File file : files) {
                    try {
                        String className = file.getName().replace(".java", "");
                        String fullClassName = "frc.robot.autonomous.autos." + className;

                        Class<?> auto = classLoader.loadClass(fullClassName);

                        if (BaseAuto.class.isAssignableFrom(auto)) {
                            BaseAuto baseAuto = (BaseAuto) auto.getDeclaredConstructor(SubsystemManager.class).newInstance(manager);
                            autoChooser.addOption(baseAuto.getName(), baseAuto);
                        }
                    } catch (Exception e) {
                        System.out.print("Auto: " + file.getName() + "could not load");
                    }
                }
            }
        }

        List<String> ppAutoNames = new ArrayList<>();
        try {   
            ppAutoNames = AutoBuilder.getAllAutoNames();
        } catch (Exception e) {
            System.err.print("PathPlannerAuto could not load");
        }

        for(String name : ppAutoNames) {
            BaseAuto auto = BaseAuto.fromPathPlannerAuto(new PathPlannerAuto(name));
            autoChooser.addOption(name, auto);
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
            return autoChooser.getSelected();
        } catch (Exception e) {
            return BaseAuto.getDefaultAuto();
        }
    }

    private Command getStartDelay() {
        return new WaitCommand(startDelayEntry.getDouble(0d));
    }
}