package it.cnr.isti.wnlab.indoornavigation.emitters;

import java.util.ArrayList;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.observers.Observer;

/**
 * Common code for all emitters.
 * @param <T> Emitted data type.
 */
public abstract class AbstractEmitter<T> implements Emitter<T> {
    protected List<Observer<T>> mObservers;

    protected AbstractEmitter() {
        mObservers = new ArrayList<>();
    }

    @Override
    public void register(Observer<T> observer) {
        mObservers.add(observer);
    }

    @Override
    public void unregister(Observer<T> observer) {
        mObservers.remove(observer);
    }

    protected void notifyObservers(T data) {
        for(Observer<T> observer : mObservers)
            observer.notify(data);
    }

}
