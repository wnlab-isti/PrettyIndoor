package it.cnr.isti.wnlab.indoornavigation.types.fingerprint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import it.cnr.isti.wnlab.indoornavigation.types.RawData;

public abstract class Fingerprint<XYPosition, T extends RawData> {

    private HashMap<XYPosition,T> map;

    protected Fingerprint() {
        map = new HashMap<>();
    }

    public List<XYPosition> findNearestK(T measurement, int k) {

        // For each registered position calculate distance between row and measurement
        ArrayList<DistancePair> distancedPositions = new ArrayList<>();
        for(Map.Entry<XYPosition,T> entry : map.entrySet()) {
            // Add results to a list
            distancedPositions.add(
                    new DistancePair(
                            entry.getKey(),distanceBetween(measurement,entry.getValue())
                    ));
        }

        // Sort per distance
        Collections.sort(distancedPositions, new Comparator<DistancePair>() {
            @Override
            public int compare(DistancePair p1, DistancePair p2) {
                return (int) (p1.distance - p2.distance); // Negative if p1 is nearer than p2
            }
        });

        // Return all or first K values
        return distancedPositions.subList(0, k-1);
    }

    public abstract float distanceBetween(T data1, T data2);

    /**
     * Private wrapper class for (position,distance) pairs.
     */
    private class DistancePair {
        public final XYPosition position;
        public final float distance;

        private DistancePair(XYPosition position, float distance) {
            this.position = position;
            this.distance = distance;
        }
    }
}
