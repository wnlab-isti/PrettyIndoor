package it.cnr.isti.wnlab.indoornavigation.observer;

import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * An observer for new raw data notifications.
 * @param <T>
 */
public interface DataObserver<T extends RawData> extends Observer<T> {}
