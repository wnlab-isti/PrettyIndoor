package it.cnr.isti.wnlab.indoornavigation.types.fingerprint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import it.cnr.isti.wnlab.indoornavigation.types.RawData;

public abstract class Fingerprint<XYPosition, T extends RawData> {

    protected HashMap<XYPosition,T> map;

    protected Fingerprint() {
        // Initialize map and populate with elements
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
                return p1.compareTo(p2); // Negative if p1 is nearer than p2
            }
        });

        for(DistancePair p : distancedPositions)
                System.out.println(p);

        // Isolate first K elements
        List<DistancePair> firstKElements;
        if(distancedPositions.size() <= k)
            firstKElements = distancedPositions;
        else
            firstKElements = distancedPositions.subList(0,k);

        // Return first K positions
        ArrayList<XYPosition> result = new ArrayList<>();
        for(DistancePair p : firstKElements)
            result.add(p.position);
        return result;
    }

    /**
     * Calculate the distance between one measurement and another.
     * @param data1
     * @param data2
     * @return The always positive distance between the two measurements.
     */
    protected abstract float distanceBetween(T data1, T data2);

    /*
     * UTILITY CLASSES
     */

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

        /**
         * @param p
         * @return a negative integer if this pair is nearer (less distanced) than the argument's.
         */
        public int compareTo(DistancePair p) {
            return (int) (distance - p.distance);
        }

        @Override
        public String toString() {
            return position + "," + distance;
        }
    }

}
