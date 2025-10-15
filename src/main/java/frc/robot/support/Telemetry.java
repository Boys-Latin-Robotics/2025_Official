package frc.robot.support;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.HashMap;
import java.util.Map;

/**
 * Telemetry provides a convenience utility around {@link DataLogManager} and {@link DataLog} in support of capturing
 * telemetry data in a uniform and consistent way. A limited subset of {@link DataLogEntry} types are currently
 * supported.
 */
public class Telemetry {

    // A registry of active StringLogEntry keyed by recorded name
    private static final Map<String, StringLogEntry> stringLogs = new HashMap<>();

    // A registry of active FloatLogEntry keyed by recorded name
    private static final Map<String, FloatLogEntry> floatLogs = new HashMap<>();

    // A registry of active DoubleLogEntry keyed by recorded name
    private static final Map<String, DoubleLogEntry> doubleLogs = new HashMap<>();

    // A registry of active IntegerLogEntry keyed by recorded name
    private static final Map<String, IntegerLogEntry> intLogs = new HashMap<>();

    public static void record(final String key, final String value) {
        requireNonNull(key, "key cannot be null");
        requireNonNull(value, "value cannot be null");
        StringLogEntry entry = stringLogs.computeIfAbsent(key, (k) -> new StringLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    public static void record(final String key, final float value) {
        requireNonNull(key, "key cannot be null");
        FloatLogEntry entry = floatLogs.computeIfAbsent(key, (k) -> new FloatLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    public static void record(final String key, final double value) {
        requireNonNull(key, "key cannot be null");
        DoubleLogEntry entry = doubleLogs.computeIfAbsent(key, (k) -> new DoubleLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    public static void record(final String key, final int value) {
        requireNonNull(key, "key cannot be null");
        IntegerLogEntry entry = intLogs.computeIfAbsent(key, (k) -> new IntegerLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    public static void info(final String value) {
        requireNonNull(value, "value cannot be null");
        DataLogManager.log(value);
    }

    public static void shutdown() {
        DataLogManager.stop();
    }
}
