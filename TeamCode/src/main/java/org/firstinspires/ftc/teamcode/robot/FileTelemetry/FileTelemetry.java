package org.firstinspires.ftc.teamcode.robot.fileTelemetry;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import org.firstinspires.ftc.teamcode.robot.fileTelemetry.formatters.CsvFormatter;
import org.firstinspires.ftc.teamcode.robot.fileTelemetry.formatters.Formatter;
import org.firstinspires.ftc.teamcode.robot.fileTelemetry.formatters.JsonFormatter;

/**
 * Writes telemetry snapshots to a file, serialized by a {@link Formatter}.
 *
 * Register fields with {@link #addField(String)} (in log order), then call
 * {@link #takeSnapshot(Object...)} once per row. The header (CSV) is written on first snapshot.
 */
public class FileTelemetry {
    public enum Type {
        CSV,
        JSON
    }

    public enum OverwriteMode {
        OVERWRITE,
        DATE_BASED,
        TIMESTAMP_BASED
    }

    private final Formatter formatter;
    private final File outputFile;
    private BufferedWriter writer;
    private boolean headerWritten = false;

    public FileTelemetry(Type type, String outputPath, OverwriteMode overwriteMode) {
        switch (type) {
            case CSV:
                formatter = new CsvFormatter();
                break;
            case JSON:
                formatter = new JsonFormatter();
                break;
            default:
                throw new IllegalArgumentException("Unsupported type: " + type);
        }

        String fileName;
        switch (overwriteMode) {
            case TIMESTAMP_BASED:
                fileName = "telemetry_" + System.currentTimeMillis() + formatter.getExtension();
                break;
            case DATE_BASED:
                String dateStr = new java.text.SimpleDateFormat("yyyy-MM-dd", java.util.Locale.getDefault())
                        .format(new java.util.Date());
                fileName = "telemetry_" + dateStr + formatter.getExtension();
                break;
            case OVERWRITE:
            default:
                fileName = "telemetry" + formatter.getExtension();
                break;
        }

        outputFile = new File(outputPath, fileName);
    }

    /** Registers a field in log order. Pass the same fields for every snapshot. */
    public void addField(String name) {
        formatter.addField(name);
    }

    public void takeSnapshot(Object... data) {
        try {
            openWriter();

            if (!headerWritten) {
                String header = formatter.formatHeader();
                if (!header.isEmpty()) {
                    writer.write(header);
                    writer.newLine();
                }
                headerWritten = true;
            }

            writer.write(formatter.formatRow(data));
            writer.newLine();
            writer.flush();
        } catch (IOException e) {
            throw new RuntimeException("Failed to write telemetry to " + outputFile, e);
        }
    }

    /** Flushes and closes the output file. Safe to call multiple times. */
    public void close() {
        if (writer != null) {
            try {
                writer.flush();
                writer.close();
            } catch (IOException ignored) {
                // best effort
            }
            writer = null;
        }
    }

    private void openWriter() throws IOException {
        if (writer == null) {
            writer = new BufferedWriter(new FileWriter(outputFile, false));
        }
    }
}
