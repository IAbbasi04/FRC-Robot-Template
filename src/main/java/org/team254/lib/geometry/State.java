package org.team254.lib.geometry;

import org.team254.lib.util.CSVWritable;
import org.team254.lib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    S add(S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
