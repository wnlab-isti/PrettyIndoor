package it.cnr.isti.wnlab.indoornavigation.emitters;

import it.cnr.isti.wnlab.indoornavigation.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigation.observers.Observer;

public interface Emitter<T> extends StartableStoppable {
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
