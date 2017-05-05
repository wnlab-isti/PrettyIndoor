package it.cnr.isti.wnlab.indoornavigation.types.fingerprint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * An object representing a generic fingerprint map with some utility methods.
 * @param <P>
 * @param <T>
 */
public abstract class FingerprintMap<P extends XYPosition, T extends RawData> {

    protected HashMap<P,T> map;

    protected FingerprintMap() {
        // Initialize map and populate with elements
        map = new HashMap<>();
    }

    /**
     * K-NN implementation.
     * @param measurement
     * @param k The "K" of K-NN
     * @param filterPolicy can be null. If it isn't, it is the policy for choosing if a fingerprint
     *                     must be evaluated or not.
     * @return a distance-ordered list of nearest K positions.
     */
    public List<PositionDistance<P>> findNearestK(
            T measurement,
            int k,
            PositionDistance.Filter filterPolicy
    ) {
        List<PositionDistance<P>> distancedPositions = getDistancedPoints(measurement, filterPolicy);

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
