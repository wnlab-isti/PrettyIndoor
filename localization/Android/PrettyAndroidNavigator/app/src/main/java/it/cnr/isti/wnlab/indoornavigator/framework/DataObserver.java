package it.cnr.isti.wnlab.indoornavigator.framework;

import it.cnr.isti.wnlab.indoornavigator.framework.types.RawData;

/**
 * Observer for a specific data type.
 * @param <T> Data type.
 */
public interface DataObserver<T extends RawData> {
    void notify(T data);
}
