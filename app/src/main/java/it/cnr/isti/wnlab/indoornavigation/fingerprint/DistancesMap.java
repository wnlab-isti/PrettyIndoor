package it.cnr.isti.wnlab.indoornavigation.fingerprint;

import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * Given a ( (x,y) , value ) fingerprint map, the instance of a DistancesMap represents the
 * collection of pairs ( (x,y) , distance ) where distances refer to the last observed measurement.
 * <p>
 * This class has a lazy policy: the map is actually made only with the first "getDistances()".
 *
 * @param <P>
 * @param <T>
 */
public class DistancesMap<P extends XYPosition, T extends RawData> implements Observer<T> {

    private FingerprintMap<P,T> fingerprintMap;
    private T lastMeasurement;
    private List<PositionDistance<P>> distanceMap;
    private PositionDistance.Filter policy;
    private int mK;

    public DistancesMap(
            FingerprintMap<P,T> fingerprintMap,
            int k,
            PositionDistance.Filter policy
    ) {
        this.fingerprintMap = fingerprintMap;
        this.policy = policy;
        this.mK = k;
    }

    public List<PositionDistance<P>> getDistances() {

        // No available measurements yet
        if(lastMeasurement == null)
            return null;

        // A new measurement arrived and the old map was invalidated
        if(distanceMap == null)
            distanceMap = fingerprintMap.findNearestK(lastMeasurement, mK, policy);

        return distanceMap;
    }

    public XYPosition findWeightedAveragePosition() {
        return findWeightedAveragePosition(getDistances());
    }

    @Override
    public void notify(T data) {
        lastMeasurement = data;
        distanceMap = null;
    }

    public static <A extends XYPosition>
    XYPosition findWeightedAveragePosition(List<PositionDistance<A>> distances) {

        if(distances != null && !distances.isEmpty()) {

            // Do weighted average
            float avgX = 0.f;
            float avgY = 0.f;
            float weightSum = 0.f;
            for (PositionDistance<A> p : distances) {
                float weight = 1 / p.distance;
                avgX += weight * p.position.x;
                avgY += weight * p.position.y;
                weightSum += weight;
            }
            avgX /= weightSum;
            avgY /= weightSum;

            // Return position
            return new XYPosition(avgX, avgY);

        } else
            return null;
    }

    @Override
    public String toString() {
        StringBuilder csv = new StringBuilder();
        csv
                .append("Measurement:").append(lastMeasurement.toString()).append("\n")
                .append("Distances:");
        for(PositionDistance<P> pd : getDistances())
            csv.append("\n").append(pd.position).append(",").append(pd.distance);

        return csv.toString();
    }
}
