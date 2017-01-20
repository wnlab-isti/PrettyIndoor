package it.cnr.isti.wnlab.indoornavigator.emitters;

import it.cnr.isti.wnlab.indoornavigator.observers.Observer;

public interface Emitter<T> {
    /**
     * Register an observer.
     * @param observer
     */
    void register(Observer<T> observer);

    /**
     * Unregister an observer.
     * @param observer
     */
    void unregister(Observer<T> observer);
}
