package it.cnr.isti.wnlab.indoornavigator.framework;

/**
 * Immutable object for an indoor position.
 */
public class IndoorPosition extends XYPosition {
    public final int floor;
    public final long timestamp;

    public IndoorPosition(float x, float y, int floor, long timestamp) {
        super(x,y);
        this.floor = floor;
        this.timestamp = timestamp;
    }

    public IndoorPosition(XYPosition p, int floor, long timestamp) {
        super(p.x, p.y);
        this.floor = floor;
        this.timestamp = timestamp;
    }
}
