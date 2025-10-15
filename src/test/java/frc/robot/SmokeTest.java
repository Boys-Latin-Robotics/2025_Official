package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** A smoke test to ensure JUnit and the test stage of our GitHub actions execute as expected */
public class SmokeTest {

    @BeforeEach
    public void setup() {
        // noop
    }

    @AfterEach
    public void tearDown() {
        // noop
    }

    @Test
    public void testSmoke() {
        assertTrue(true);
    }

    @Test
    public void testSmoke2() {
        assertTrue(true);
    }
}
