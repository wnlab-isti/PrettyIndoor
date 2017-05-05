package it.cnr.isti.wnlab.indoornavigation.observer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Usual code common to all emitters (observers management, etc...) with a lazy policy.
 * <p>
 * Note it is true that:
 * LazyEmitter is active <=> Exists at least 1 registered observer
 *
 * @param <T> Emitted data type.
 */
public abstract class LazyEmitter<T> implements Emitter<T> {
    protected List<Observer<T>> mObservers;

    protected LazyEmitter() {
        mObservers = new ArrayList<>();
    }

    @Override
    public void register(Observer<T> observer) {
        if(observer != null) {
            // Save if it is empty now or not
            boolean wasEmpty = mObservers.isEmpty();

            // Add observer
            mObservers.add(observer);

            // Start if needed
            checkStart(wasEmpty);
        }
    }

    @Override
    public void register(Collection<Observer<T>> observers) {
        if(observers != null) {
            // Save if it is empty now or not
            boolean wasEmpty = mObservers.isEmpty();

            // Add observers
            mObservers.addAll(observers);

            // Start if needed
            checkStart(wasEmpty);
        }
    }

    private void checkStart(boolean wasEmpty) {
        if(wasEmpty && !mObservers.isEmpty()) {
            // System.out.println(this.getClass().getCanonicalName().toString() + " is starting");
            startEmission();
        }
    }

    @Override
    public void unregister(Observer<T> observer) {
        if(observer != null) {
            // Save if it is empty now or not
            boolean wasEmpty = mObservers.isEmpty();

            // Remove observer
            mObservers.remove(observer);

            // Stop if needed
            checkStop(wasEmpty);
        }
    }

    @Override
    public List<Observer<T>> unregisterAll() {
        // Save if it is empty now or not
        checkStop(!mObservers.isEmpty());

        // Save current observers list to return and reinitialize this' observers list
        List<Observer<T>> observers = mObservers;
        mObservers = new ArrayList<>();

        return observers;

    }

    private void checkStop(boolean wasEmpty) {
        if(!wasEmpty && mObservers.isEmpty()) {
            System.out.println(this.getClass().getCanonicalName().toString() + " is stopping");
            stopEmission();
        }
    }

    protected void notifyObservers(T data) {
        for(Observer<T> observer : mObservers)
            observer.notify(data);
    }

    /**
     * Actually starts the emission.
     */
    protected abstract void startEmission();

    /**
     * Actually stops the emission.
     */
    protected abstract void stopEmission();

}
