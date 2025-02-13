package org.team8592.lib.field;

import edu.wpi.first.apriltag.AprilTagFields;

public class ReefscapeFieldLayout extends FieldLayout {
    public ReefscapeFieldLayout() {
        super(AprilTagLayout.fromYear(AprilTagFields.k2025Reefscape));
    }
}
