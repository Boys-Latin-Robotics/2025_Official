package frc.robot.support.limelight;

import static java.lang.String.format;

import edu.wpi.first.net.PortForwarder;
import frc.robot.support.Telemetry;

/**
 * A collection of Limelight related utilities
 */
public class LimelightUtil {
    private static final String DEFAULT_HOSTNAME = "limelight.local";
    public static final int LOW_PORT = 5800;
    public static final int HIGH_PORT = 5809;

    public static void startPortForwarding() {
        startPortForwarding(DEFAULT_HOSTNAME);
    }

    public static void startPortForwarding(final String hostname) {
        for (int port = LOW_PORT; port <= HIGH_PORT; port++) {
            PortForwarder.add(port, hostname, port);
            Telemetry.info(format("Limelight now forwarding to %s:%s", hostname, port));
        }
    }

    public static void stopPortForwarding() {
        for (int port = LOW_PORT; port <= HIGH_PORT; port++) {
            PortForwarder.remove(port);
        }
    }
}
