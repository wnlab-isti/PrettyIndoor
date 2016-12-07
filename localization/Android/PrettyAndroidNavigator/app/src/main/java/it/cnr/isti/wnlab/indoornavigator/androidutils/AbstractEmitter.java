package it.cnr.isti.wnlab.indoornavigator.androidutils;

import java.util.ArrayList;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigator.framework.DataEmitter;
import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.types.RawData;

/**
 * Common code for all emitters.
 * @param <T> Emitted data type.
 */
public abstract class AbstractEmitter<T extends RawData> implements DataEmitter<T> {
    protected List<DataObserver<T>> mObservers;

    protected AbstractEmitter() {
        mObservers = new ArrayList<>();
    }

    @Override
    public void register(DataObserver<T> observer) {
        mObservers.add(observer);
    }

    @Override
    public void unregister(DataObserver<T> observer) {
        mObservers.remove(observer);
    }

    protected void notifyToAll(T data) {
        for(DataObserver<T> observer : mObservers)
            observer.notify(data);
    }

}
