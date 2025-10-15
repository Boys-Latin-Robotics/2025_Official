package frc.robot.support;

/**
 * An enumeration of available DIO channels. This class should be used in place of raw numerical values throughout
 * configurations and settings to assist in keeping track of which channels are used where.
 */
public enum DIOChannel {
    ZERO(0),
    ONE(1),
    TWO(2),
    THREE(3),
    FOUR(4),
    FIVE(5),
    SIX(6),
    SEVEN(7),
    EIGHT(8),
    NINE(9);

    DIOChannel(int channel) {
        this.channel = channel;
    }

    private final int channel;

    public int getChannel() {
        return this.channel;
    }
}
