package it.cnr.isti.wnlab.indoornavigation.observer;

import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * An emitter for new come raw data.
 * @param <T>
 */
public abstract class DataEmitter<T extends RawData>
        extends LazyEmitter<T> {}
