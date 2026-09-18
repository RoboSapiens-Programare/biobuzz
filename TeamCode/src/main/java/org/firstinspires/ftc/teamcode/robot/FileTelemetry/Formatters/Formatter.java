package org.firstinspires.ftc.teamcode.robot.fileTelemetry.formatters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Base template for telemetry row formatters. Subclasses serialize one snapshot of registered
 * fields into their own text format (e.g. CSV or JSON Lines).
 *
 * Register fields in the exact order they will be logged with {@link #addField(String)}, then
 * feed the matching values to {@link #formatRow(Object...)}.
 */
public abstract class Formatter {
    private final List<String> fields = new ArrayList<>();

    /**
     * Registers a field if not already present. Field order is preserved.
     *
     * @return true if the field was added, false if it was already registered.
     */
    public boolean addField(String name) {
        if (name == null) {
            return false;
        }

        for (String field : fields) {
            if (name.equals(field)) {
                return false;
            }
        }

        fields.add(name);
        return true;
    }

    /** Immutable view of the registered fields, in registration order. */
    public List<String> getFields() {
        return Collections.unmodifiableList(fields);
    }

    /** Optional header line emitted before the first row. Empty by default. */
    public String formatHeader() {
        return "";
    }

    /**
     * Serializes one snapshot row.
     *
     * @param values one value per registered field, in field order.
     */
    public abstract String formatRow(Object... values);

    /** File extension including the dot, e.g. ".csv" or ".json". */
    public abstract String getExtension();

    /** Guards against value/field count mismatches. */
    protected void checkSize(Object[] values) {
        if (values.length != fields.size()) {
            throw new IllegalArgumentException(
                    "Expected " + fields.size() + " values but got " + values.length
                            + " for fields " + fields);
        }
    }
}