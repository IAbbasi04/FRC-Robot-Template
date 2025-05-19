package org.team1731.lib;

import java.util.List;

public class EnumUtils {
    public static <T extends Enum<T>> List<T> getAsList(Class<T> enumClass) {
        T[] constants = enumClass.getEnumConstants();
        return List.of(constants);
    }
}