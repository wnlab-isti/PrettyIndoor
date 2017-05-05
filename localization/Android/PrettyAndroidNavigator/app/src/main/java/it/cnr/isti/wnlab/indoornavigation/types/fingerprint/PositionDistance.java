package it.cnr.isti.wnlab.indoornavigation.types.fingerprint;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;

/**
 * Wrapper class for (position,distance) pairs.
 */
public class PositionDistance<P extends XYPosition> {

    public final P position;
    public final float distance;

    public PositionDistance(P position, float distance) {
        this.position = position;
        this.distance = distance;
    }

    /**
     * Rule for comparison is:
     * (p1,d1) <= (p2,d2) <=> d1 <= d2 <=> p1.compareTo(p2) < 0
     * @param p
     * @return a negative integer if this pair is nearer (less distanced) than the argument's.
     */
    public int compareTo(PositionDistance p) {
        return (int) (distance - p.distance);
    }

    @Override
    public String toString() {
        return position + "," + distance;
    }

    public interface Filter {
        boolean isValid(XYPosition position, float distance);
    }
}