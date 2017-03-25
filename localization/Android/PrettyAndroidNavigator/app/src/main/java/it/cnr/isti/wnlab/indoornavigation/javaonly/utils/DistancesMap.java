package it.cnr.isti.wnlab.indoornavigation.javaonly.utils;

import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.FingerprintMap;

public class DistancesMap<P extends XYPosition, T extends RawData> {

    private FingerprintMap<P,T> fingerprintMap;
    private T lastMeasurement;
    private List<FingerprintMap.PositionDistance<P>> distanceMap;
    private float distanceThreshold;
    private int mK;

    public DistancesMap(
            FingerprintMap<P,T> fingerprintMap,
            DataEmitter<T> emitter,
            int k,
            float distanceThreshold
    ) {
        this.fingerprintMap = fingerprintMap;
        this.distanceThreshold = distanceThreshold;
        this.mK = k;
        // Update last measurement and invalidate last map
        emitter.register(new Observer<T>() {
            @Override
            public void notify(T data) {
                lastMeasurement = data;
                distanceMap = null;
            }
        });
    }

    public List<FingerprintMap.PositionDistance<P>> getDistances() {
        // No available measurements yet
        if(lastMeasurement == null)
            return null;

        // A new measurement arrived and the old map was invalidated
        if(distanceMap == null)
            distanceMap = fingerprintMap.findNearestK(lastMeasurement, mK, distanceThreshold);

        return distanceMap;
    }

    public XYPosition findAveragePosition() {
        if(distanceMap != null) {
            // Do weighted average
            float avgX = 0.f;
            float avgY = 0.f;
            float weightSum = 0.f;
            for (FingerprintMap.PositionDistance<P> p : distanceMap) {
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
        for(FingerprintMap.PositionDistance<P> pd : getDistances())
            csv.append("\n").append(pd.position).append(",").append(pd.distance);

        return csv.toString();
    }

}
