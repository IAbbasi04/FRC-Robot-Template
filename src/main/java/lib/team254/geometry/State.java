package lib.team254.geometry;

import lib.team254.util.CSVWritable;
import lib.team254.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    S add(S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
