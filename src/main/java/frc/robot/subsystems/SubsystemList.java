package frc.robot.subsystems;

import java.util.ArrayList;

public class SubsystemList extends ArrayList<Subsystem<?>> {
    public SubsystemList(Subsystem<?>... subsystems) {
        for (Subsystem<?> subsystem : subsystems) {
            this.add(subsystem);
        }
    }
}