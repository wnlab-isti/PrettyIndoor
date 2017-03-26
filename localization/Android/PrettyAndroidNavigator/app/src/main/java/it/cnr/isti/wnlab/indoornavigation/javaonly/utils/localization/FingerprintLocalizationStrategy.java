package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.FingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.PositionDistance;

public abstract class FingerprintLocalizationStrategy<T extends RawData>
        extends AbstractEmitter<XYPosition> {

    private FingerprintMap<XYPosition,T> fpMap;
    private int mK;

    public FingerprintLocalizationStrategy(
            FingerprintMap<XYPosition, T> fingerprintMap,
            DataEmitter<T> dataEmitter,
            int k
    ) {
        fpMap = fingerprintMap;
        dataEmitter.register(new Observer<T>() {
            @Override
            public void notify(T data) {
                notifyObservers(findAveragePosition(data));
            }
        });
        mK = k;
    }

    private XYPosition findAveragePosition(T lastMeasurement) {
        // Get first K positions
        Collection<PositionDistance<XYPosition>> firstKPositions;
        firstKPositions = fpMap.findNearestK(lastMeasurement,mK,null);

        // Do weighted average
        float avgX = 0.f;
        float avgY = 0.f;
        float weightSum = 0.f;
        for(PositionDistance<XYPosition> p : firstKPositions) {
            float weight = 1/p.distance;
            avgX += weight * p.position.x;
            avgY += weight * p.position.y;
            weightSum += weight;
        }
        avgX /= weightSum;
        avgY /= weightSum;

        // Return position
        return new XYPosition(avgX, avgY);
    }


}
