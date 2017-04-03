package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import org.apache.commons.math3.exception.NullArgumentException;

import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.javaonly.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.DistancesMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.FingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.PositionDistance;

public class SimpleFingerprintLocalization<T extends RawData>
        extends AbstractIndoorLocalizationStrategy
        implements DataObserver<T> {

    private FloorMap floor;
    private XYPosition position;
    private FingerprintMap<XYPosition,T> fpMap;
    private int k;
    private PositionDistance.Filter filter;
    private Emitter<T> emitter;

    public SimpleFingerprintLocalization(
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
