package it.cnr.isti.wnlab.indoornavigation.utils.localization.fingerprint;

import org.apache.commons.math3.exception.NullArgumentException;

import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.DistancesMap;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.FingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.PositionDistance;

/**
 * A not-so-smart localization algorithm that uses only a fingerprint map.
 * @param <T>
 */
public class FingerprintStrategy<T extends RawData>
        extends AbstractIndoorLocalizationStrategy
        implements DataObserver<T> {

    private FloorMap floor;
    private XYPosition position;
    private FingerprintMap<XYPosition,T> fpMap;
    private int k;
    private PositionDistance.Filter filter;
    private Emitter<T> emitter;

    public FingerprintStrategy(
            FloorMap floor,
            FingerprintMap<XYPosition,T> fpMap,
            Emitter<T> emitter,
            int k, PositionDistance.Filter filter) {
        if(floor == null || fpMap == null || emitter == null)
            throw new NullArgumentException();
        this.floor = floor;
        this.fpMap = fpMap;
        this.filter = filter;
        this.k = k;
        this.emitter = emitter;
    }

    @Override
    public IndoorPosition getCurrentPosition() {
        return new IndoorPosition(position,floor.getFloor(),System.currentTimeMillis());
    }

    @Override
    protected void startEmission() {
        emitter.register(this);
    }

    @Override
    protected void stopEmission() {
        emitter.unregister(this);
    }

    @Override
    public void notify(T data) {
        List<PositionDistance<XYPosition>> positions = fpMap.findNearestK(data, k, filter);
        XYPosition newPosition = DistancesMap.findWeightedAveragePosition(positions);
        if(floor != null)
            newPosition = floor.nearestValid(newPosition.x,newPosition.y);
        position = newPosition;
        notifyObservers(new IndoorPosition(position,floor.getFloor(),System.currentTimeMillis()));
    }
}
