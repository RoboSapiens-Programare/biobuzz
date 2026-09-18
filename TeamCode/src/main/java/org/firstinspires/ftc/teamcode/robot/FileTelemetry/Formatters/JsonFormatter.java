package org.firstinspires.ftc.teamcode.robot.fileTelemetry.formatters;

import com.google.gson.Gson;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Formats telemetry rows as JSON Lines (one flat JSON object per snapshot).
 * Values keep their runtime type, so numbers stay numbers, booleans stay booleans and
 * enums serialize as their name (e.g. "SPOOLING").
 */
public class JsonFormatter extends Formatter {
    private final Gson gson = new Gson();

    @Override
    public String formatRow(Object... values) {
        checkSize(values);

        Map<String, Object> row = new LinkedHashMap<>();
        for (int i = 0; i < values.length; i++) {
            row.put(getFields().get(i), values[i]);
        }
        return gson.toJson(row);
    }

    @Override
    public String getExtension() {
        return ".json";
    }
}
