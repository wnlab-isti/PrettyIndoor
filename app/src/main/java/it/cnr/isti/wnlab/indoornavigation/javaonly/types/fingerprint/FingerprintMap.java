package it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint;

import android.util.Log;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;

public abstract class FingerprintMap<P extends XYPosition, T extends RawData> {

    protected HashMap<P,T> map;

    protected FingerprintMap() {
        // Initialize map and populate with elements
        map = new HashMap<>();
    }

    public List<PositionDistance<P>> findNearestK(T measurement, int k, PositionDistance.Filter policy) {
        List<PositionDistance<P>> distancedPositions = getDistancedPoints(measurement, policy);

        // Sort per distance
        Collections.sort(distancedPositions, new Comparator<PositionDistance>() {
            @Override
            public int compare(PositionDistance p1, PositionDistance p2) {
                return p1.compareTo(p2); // Negative if p1 is nearer than p2
            }
        });

        // Isolate first K elements
        List<PositionDistance<P>> firstKElements;
        if(distancedPositions.size() <= k)
            firstKElements = distancedPositions;
        else
            firstKElements = distancedPositions.subList(0,k);

        return firstKElements;
    }

    /**
     * @param measurement
     * @param policy the acceptance policy for positions. If null every point will be accepted.
     * @return An unordered list of all points in database with their row-distances from measurement.
     */
    public List<PositionDistance<P>> getDistancedPoints(T measurement, PositionDistance.Filter policy) {
        // For each registered position calculate distance between row and measurement
        ArrayList<PositionDistance<P>> distancedPositions = new ArrayList<>();
        for(Map.Entry<P,T> entry : map.entrySet()) {
            // Add results to a list if distance is acceptable
            float distance = distanceBetween(measurement,entry.getValue());
            if(policy == null || policy.isValid(entry.getKey(),distance))
                distancedPositions.add(
                        new PositionDistance(entry.getKey(), distance));
        }

        return distancedPositions;
    }

    /**
     * Calculate the distance between one measurement and another.
     * @param data1
     * @param data2
     * @return The always positive distance between the two measurements.
     */
    protected abstract float distanceBetween(T data1, T data2);
}
