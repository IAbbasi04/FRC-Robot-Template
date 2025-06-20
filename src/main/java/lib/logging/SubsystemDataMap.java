package lib.logging;

import java.util.HashMap;
import java.util.Map;

public class SubsystemDataMap<E extends Enum<E>> {
    private Map<E, DataEntry<?>> dataMap = new HashMap<E, DataEntry<?>>();

    public SubsystemDataMap() {}

    public <T> void set(E key, T value) {
        dataMap.put(key, new DataEntry<T>(() -> value));
    }

    public <T> void setIf(E key, T value, T alternative, boolean condition) {
        if (condition) {
            dataMap.put(key, new DataEntry<T>(() -> value));
        } else {
            dataMap.put(key, new DataEntry<T>(() -> alternative));
        }
    }

    @SuppressWarnings("unchecked")
    public <T> T get(E key) {
        try {
            return (T)dataMap.get(key).get();
        } catch (Exception e) {
            return null;
        }
    }
}