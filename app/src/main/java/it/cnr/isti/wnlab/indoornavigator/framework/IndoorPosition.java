package it.cnr.isti.wnlab.indoornavigator.framework;

/**
 * Immutable object for an indoor position.
 */
public class IndoorPosition {
    public final float x, y;
    public final int floor;
    public final long timestamp;

    public IndoorPosition(float x, float y, int floor, long timestamp) {
        this.x = x;
        this.y = y;
        this.floor = floor;
        this.timestamp = timestamp;
    }
}
