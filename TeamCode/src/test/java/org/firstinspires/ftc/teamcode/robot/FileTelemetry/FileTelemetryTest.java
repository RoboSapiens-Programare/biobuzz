package org.firstinspires.ftc.teamcode.robot.FileTelemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.file.Files;
import java.nio.file.Path;
import org.json.JSONArray;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class FileTelemetryTest {
    @Test
    void csvWritesHeaderOnceAndAppendsRows(@TempDir Path tempDir) throws Exception {
        java.io.File file = tempDir.resolve("telemetry.csv").toFile();
        FileTelemetry telemetry = new FileTelemetry(FileTelemetry.Type.CSV, file);
        telemetry.addField("action");
        telemetry.addField("value");

        telemetry.takeSnapshot("a", 1);
        telemetry.takeSnapshot("b", 2);
        telemetry.close();

        assertEquals(
                "action,value\na,1\nb,2\n",
                new String(Files.readAllBytes(file.toPath()), java.nio.charset.StandardCharsets.UTF_8));
        assertEquals(2, telemetry.getSnapshotCount());
    }

    @Test
    void csvTruncatesExistingFile(@TempDir Path tempDir) throws Exception {
        java.io.File file = tempDir.resolve("telemetry.csv").toFile();
        Files.write(file.toPath(), "old data\n".getBytes(java.nio.charset.StandardCharsets.UTF_8));

        FileTelemetry telemetry = new FileTelemetry(FileTelemetry.Type.CSV, file);
        telemetry.addField("action");
        telemetry.takeSnapshot("new");
        telemetry.close();

        assertEquals(
                "action\nnew\n",
                new String(Files.readAllBytes(file.toPath()), java.nio.charset.StandardCharsets.UTF_8));
    }

    @Test
    void jsonArrayAppendsAndReloadsRows(@TempDir Path tempDir) throws Exception {
        java.io.File file = tempDir.resolve("telemetry.json").toFile();
        FileTelemetry telemetry = new FileTelemetry(FileTelemetry.Type.JSON_ARRAY, file);
        telemetry.addField("action");
        telemetry.addField("value");

        telemetry.takeSnapshot("shoot", 1.5);
        telemetry.takeSnapshot("drive", 2.0);

        JSONArray array =
                new JSONArray(new String(Files.readAllBytes(file.toPath()), java.nio.charset.StandardCharsets.UTF_8));
        assertEquals(2, array.length());
        assertEquals("shoot", array.getJSONObject(0).getString("action"));
        assertEquals(2.0, array.getJSONObject(1).getDouble("value"), 1e-12);
        assertEquals(2, telemetry.getSnapshotCount());
    }

    @Test
    void jsonArrayRecoversFromMalformedFile(@TempDir Path tempDir) throws Exception {
        java.io.File file = tempDir.resolve("telemetry.json").toFile();
        Files.write(file.toPath(), "not json".getBytes(java.nio.charset.StandardCharsets.UTF_8));

        FileTelemetry telemetry = new FileTelemetry(FileTelemetry.Type.JSON_ARRAY, file);
        telemetry.addField("action");
        telemetry.takeSnapshot("shoot");

        JSONArray array =
                new JSONArray(new String(Files.readAllBytes(file.toPath()), java.nio.charset.StandardCharsets.UTF_8));
        assertEquals(1, array.length());
        assertEquals("shoot", array.getJSONObject(0).getString("action"));
    }
}
