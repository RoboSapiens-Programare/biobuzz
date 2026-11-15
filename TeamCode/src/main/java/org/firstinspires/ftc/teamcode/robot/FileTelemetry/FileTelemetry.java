package org.firstinspires.ftc.teamcode.robot.FileTelemetry;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;
import org.firstinspires.ftc.teamcode.robot.FileTelemetry.Formatters.CsvFormatter;
import org.firstinspires.ftc.teamcode.robot.FileTelemetry.Formatters.JsonFormatter;
import org.firstinspires.ftc.teamcode.robot.fileTelemetry.formatters.Formatter;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Writes telemetry snapshots to a file, serialized by a {@link Formatter}.
 *
 * Register fields with {@link #addField(String)} (in log order), then call
 * {@link #takeSnapshot(Object...)} once per row. The header (CSV) is written on first snapshot.
 * JSON_ARRAY snapshots are appended to a single JSON array and rewritten after each snapshot.
 */
public class FileTelemetry {
    public enum Type {
        CSV,
        JSON,
        JSON_ARRAY
    }

    public enum OverwriteMode {
        OVERWRITE,
        DATE_BASED,
        TIMESTAMP_BASED
    }

    private final Formatter formatter;
    private final File outputFile;
    private final boolean jsonArray;

    private BufferedWriter writer;
    private boolean headerWritten = false;
    private JSONArray array;
    private int snapshotCount = 0;

    public FileTelemetry(Type type, String outputPath, OverwriteMode overwriteMode) {
        this(type, new File(outputPath, defaultFileName(type, overwriteMode)));
    }

    public FileTelemetry(Type type, File outputFile) {
        formatter = createFormatter(type);
        jsonArray = type == Type.JSON_ARRAY;
        this.outputFile = outputFile;
    }

    /** Registers a field in log order. Pass the same fields for every snapshot. */
    public void addField(String name) {
        formatter.addField(name);
    }

    public void takeSnapshot(Object... data) {
        if (jsonArray) {
            takeArraySnapshot(data);
            return;
        }

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
            snapshotCount++;
        } catch (IOException e) {
            throw new RuntimeException("Failed to write telemetry to " + outputFile, e);
        }
    }

    public int getSnapshotCount() {
        if (jsonArray && array == null) {
            array = loadArray();
        }
        return snapshotCount;
    }

    public File getOutputFile() {
        return outputFile;
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

    private void takeArraySnapshot(Object... data) {
        try {
            if (array == null) {
                array = loadArray();
            }

            array.put(new JSONObject(formatter.formatRow(data)));

            try (FileWriter writer = new FileWriter(outputFile, false)) {
                writer.write(array.toString(2));
            }

            snapshotCount = array.length();
        } catch (IOException | JSONException e) {
            throw new RuntimeException("Failed to write telemetry to " + outputFile, e);
        }
    }

    private JSONArray loadArray() {
        if (!outputFile.exists()) {
            return new JSONArray();
        }

        try (Scanner scanner = new Scanner(outputFile)) {
            StringBuilder contents = new StringBuilder();
            while (scanner.hasNextLine()) {
                contents.append(scanner.nextLine());
            }
            if (contents.length() > 0) {
                JSONArray loaded = new JSONArray(contents.toString());
                snapshotCount = loaded.length();
                return loaded;
            }
        } catch (Exception ignored) {
            // Start a fresh array if the existing file is not a JSON array.
        }
        return new JSONArray();
    }

    private void openWriter() throws IOException {
        if (writer == null) {
            writer = new BufferedWriter(new FileWriter(outputFile, false));
        }
    }

    private static Formatter createFormatter(Type type) {
        switch (type) {
            case CSV:
                return new CsvFormatter();
            case JSON:
            case JSON_ARRAY:
                return new JsonFormatter();
            default:
                throw new IllegalArgumentException("Unsupported type: " + type);
        }
    }

    private static String defaultFileName(Type type, OverwriteMode overwriteMode) {
        String extension = type == Type.CSV ? ".csv" : ".json";

        switch (overwriteMode) {
            case TIMESTAMP_BASED:
                return "telemetry_" + System.currentTimeMillis() + extension;
            case DATE_BASED:
                String dateStr = new java.text.SimpleDateFormat("yyyy-MM-dd", java.util.Locale.getDefault())
                        .format(new java.util.Date());
                return "telemetry_" + dateStr + extension;
            case OVERWRITE:
            default:
                return "telemetry" + extension;
        }
    }
}
