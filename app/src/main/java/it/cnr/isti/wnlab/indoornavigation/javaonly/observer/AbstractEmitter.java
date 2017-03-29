package it.cnr.isti.wnlab.indoornavigation.javaonly.observer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

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
        boolean wasEmpty = mObservers.isEmpty();
        if(wasEmpty && mObservers.add(observer))
            start();
    }

    @Override
    public void unregister(Observer<T> observer) {
        if(mObservers.remove(observer) && mObservers.isEmpty())
            stop();
    }

    @Override
    public void register(Collection<Observer<T>> observers) {
        boolean wasEmpty = mObservers.isEmpty();
        if(wasEmpty && mObservers.addAll(observers))
            start();
    }

    @Override
    public List<Observer<T>> unregisterAll() {
        List<Observer<T>> observers = mObservers;
        mObservers = new ArrayList<>();
        return observers;
    }

    protected void notifyObservers(T data) {
        for(Observer<T> observer : mObservers)
            observer.notify(data);
    }

    protected abstract void start();

    protected abstract void stop();

}
