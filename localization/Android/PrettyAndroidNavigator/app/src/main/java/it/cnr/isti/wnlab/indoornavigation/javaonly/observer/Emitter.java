package it.cnr.isti.wnlab.indoornavigation.javaonly.observer;

import java.util.Collection;
import java.util.List;

public interface Emitter<T> {
    /**
     * Registers an observer.
     * @param observer
     */
    void register(Observer<T> observer);

    /**
     * Registers some observers.
     * @param observers
     */
    void register(Collection<Observer<T>> observers);

    /**
     * Unregisters an observer.
     * @param observer
     */
    void unregister(Observer<T> observer);

    /**
     * Unregisgters all observers.
     * @return
     */
    List<Observer<T>> unregisterAll();
}
