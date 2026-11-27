package org.firstinspires.ftc.teamcode.robot.FileTelemetry.Formatters;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class FormatterTest {
    @Test
    void csvFormatterPreservesFieldOrderAndValues() {
        CsvFormatter formatter = new CsvFormatter();
        assertTrue(formatter.addField("action"));
        assertFalse(formatter.addField("action"));
        formatter.addField("distance");

        assertEquals("action,distance", formatter.formatHeader());
        assertEquals("shoot,1.5", formatter.formatRow("shoot", 1.5));
        assertEquals(".csv", formatter.getExtension());
    }

    @Test
    void jsonFormatterKeepsNumericTypes() {
        JsonFormatter formatter = new JsonFormatter();
        formatter.addField("distance");
        formatter.addField("ready");

        assertEquals("{\"distance\":2.25,\"ready\":true}", formatter.formatRow(2.25, true));
        assertEquals(".json", formatter.getExtension());
    }

    @Test
    void formatterRejectsValueCountMismatch() {
        CsvFormatter formatter = new CsvFormatter();
        formatter.addField("action");

        assertThrows(IllegalArgumentException.class, () -> formatter.formatRow("shoot", 1.0));
    }

    @Test
    void fieldListIsUnmodifiable() {
        CsvFormatter formatter = new CsvFormatter();
        formatter.addField("action");

        assertThrows(
                UnsupportedOperationException.class, () -> formatter.getFields().add("extra"));
        assertEquals(List.of("action"), formatter.getFields());
    }
}
