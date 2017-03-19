package it.cnr.isti.wnlab.indoornavigation.javaonly.observer;

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
