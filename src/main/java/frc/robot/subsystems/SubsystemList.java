package frc.robot.subsystems;

import java.util.ArrayList;

/**
 * Helper class that extends from Arraylist to more easily manage subsystems
 */
public class SubsystemList extends ArrayList<BaseSubsystem<?, ?>> {
    public SubsystemList(BaseSubsystem<?, ?>... subsystems) {
        for (BaseSubsystem<?, ?> subsystem : subsystems) {
            this.add(subsystem);
        }
    }
}