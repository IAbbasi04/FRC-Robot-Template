package lib.logging;

import java.util.function.Supplier;

public class DataEntry<T> {
    private Supplier<T> data;

    public DataEntry(Supplier<T> data) {
        this.data = data;
    }

    public Class<?> getDataClass() {
        return data.get().getClass();
    }

    public double getDouble() {
        return (double)data.get();
    }

    public boolean getBoolean() {
        return (boolean)data.get();
    }

    public String getString() {
        return (String)data.get();
    }

    public T get() {
        return (T)data.get();
    }

    public T getEnum() {
        return (T)data.get();
    }

    @SuppressWarnings("unchecked")
    public <E extends Enum<E>> int getEnumOrdinal() {
        return ((E)data.get()).ordinal();
    }

    public String getEnumName() {
        return getEnum().toString();
    }
}