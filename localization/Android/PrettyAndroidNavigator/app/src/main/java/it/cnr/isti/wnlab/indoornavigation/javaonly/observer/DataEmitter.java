package it.cnr.isti.wnlab.indoornavigation.javaonly.observer;

import it.cnr.isti.wnlab.indoornavigation.javaonly.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;

public abstract class DataEmitter<T extends RawData>
        extends AbstractEmitter<T>
        implements StartableStoppable {}
