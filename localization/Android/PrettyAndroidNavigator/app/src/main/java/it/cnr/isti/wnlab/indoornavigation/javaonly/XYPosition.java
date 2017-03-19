package it.cnr.isti.wnlab.indoornavigation.javaonly;

import java.util.Collection;

public class XYPosition {
    public final float x;
    public final float y;

    public XYPosition(float x, float y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return x + "," + y;
    }

    /**
     * Utility method for getting the "position in the middle" using euclidean average through
     * coordinates.
     * @param positions
     * @return a "euclidean averaged" position.
     */
    public static XYPosition getAveragePosition(Collection<XYPosition> positions) {
        float avgX = 0.f;
        float avgY = 0.f;
        int n = 0;

        for(XYPosition p : positions) {
            avgX += p.x;
            avgY += p.y;
            n++;
        }

        return new XYPosition(avgX/n, avgY/n);
    }
}
