package it.cnr.isti.wnlab.indoornavigator.framework;

import it.cnr.isti.wnlab.indoornavigator.framework.types.RawData;

/**
 * Emits a data type and accepts subscribers.
 * @param <T> Emitted data type.
 */
public interface DataEmitter<T extends RawData> extends StoppableStartable {
    /**
     * Register an observer.
     * @param observer
     */
    void register(DataObserver<T> observer);

    /**
     * Unregister an observer.
     * @param observer
     */
    void unregister(DataObserver<T> observer);
}
