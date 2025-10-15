package frc.robot.support;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

public class TelemetryTest {

    @Test
    public void testTelemetryInfo() {
        try (MockedStatic<DataLogManager> util = Mockito.mockStatic(DataLogManager.class)) {
            Telemetry.info("Test");
            util.verify(() -> DataLogManager.log("Test"));
        }
    }

    @Test
    void testTelemetryRecordString() {
        try (MockedStatic<DataLogManager> util = Mockito.mockStatic(DataLogManager.class)) {
            DataLog mockedDataLog = mock(DataLog.class);
            util.when(DataLogManager::getLog).thenReturn(mockedDataLog);
            Telemetry.record("test/telemetry", "Test");
            verify(mockedDataLog).appendString(0, "Test", 0L);
        }
    }

    @Test
    void testTelemetryRecordFloat() {
        try (MockedStatic<DataLogManager> util = Mockito.mockStatic(DataLogManager.class)) {
            DataLog mockedDataLog = mock(DataLog.class);
            util.when(DataLogManager::getLog).thenReturn(mockedDataLog);
            Telemetry.record("test/telemetry", 100f);
            verify(mockedDataLog).appendFloat(0, 100f, 0L);
        }
    }

    @Test
    void testTelemetryRecordDouble() {
        try (MockedStatic<DataLogManager> util = Mockito.mockStatic(DataLogManager.class)) {
            DataLog mockedDataLog = mock(DataLog.class);
            util.when(DataLogManager::getLog).thenReturn(mockedDataLog);
            Telemetry.record("test/telemetry", 100d);
            verify(mockedDataLog).appendDouble(0, 100d, 0L);
        }
    }

    @Test
    void testTelemetryRecordInteger() {
        try (MockedStatic<DataLogManager> util = Mockito.mockStatic(DataLogManager.class)) {
            DataLog mockedDataLog = mock(DataLog.class);
            util.when(DataLogManager::getLog).thenReturn(mockedDataLog);
            Telemetry.record("test/telemetry", 100);
            verify(mockedDataLog).appendInteger(0, 100, 0L);
        }
    }
}
