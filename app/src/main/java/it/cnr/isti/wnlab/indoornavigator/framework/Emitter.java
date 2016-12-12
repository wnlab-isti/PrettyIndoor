package it.cnr.isti.wnlab.indoornavigator.framework;

/**
 * Created by m on 09/12/16.
 */

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
