package org.firstinspires.ftc.teamcode.robot.fileTelemetry.formatters;

import java.util.StringJoiner;

/** Formats telemetry rows as comma-separated values, one row per snapshot. */
public class CsvFormatter extends Formatter {

    @Override
    public String formatHeader() {
        return String.join(",", getFields());
    }

    @Override
    public String formatRow(Object... values) {
        checkSize(values);

        StringJoiner joiner = new StringJoiner(",");
        for (Object value : values) {
            joiner.add(value == null ? "" : value.toString());
        }
        return joiner.toString();
    }

    @Override
    public String getExtension() {
        return ".csv";
    }
}