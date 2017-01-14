package it.cnr.isti.wnlab.indoornavigator.utils;

import java.util.Collection;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigator.XYPosition;

public final class Utils {

    private Utils() {}

    /**
     * Calculate average position.
     * @param positions
     * @return
     */
    public static XYPosition averagePosition(Collection<XYPosition> positions) {
        float[] newXY = {0.f, 0.f};
        for (XYPosition p : positions) {
            newXY[0] += p.x;
            newXY[1] += p.y;
        }
        newXY[0] /= positions.size();
        newXY[1] /= positions.size();

        return new XYPosition(newXY[0], newXY[1]);
    }

    public static List<XYPosition> knn(final List<XYPosition> positions, final List<Float> distances, int k) {
        // TODO
        return positions;
    }
}
