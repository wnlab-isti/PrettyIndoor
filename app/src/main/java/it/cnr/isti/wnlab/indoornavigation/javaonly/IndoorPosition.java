package it.cnr.isti.wnlab.indoornavigation.javaonly;

import java.io.Serializable;

/**
 * Immutable object for an indoor position.
 */
public class IndoorPosition extends XYPosition implements Serializable {
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

    /**
     * @return StepLogger-like formatted position information.
     */
    @Override
    public String toString() {
        return timestamp + "," + super.toString() + "," + floor;
    }
}
