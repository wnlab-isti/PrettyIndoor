package it.cnr.isti.wnlab.indoornavigation.observer;

import it.cnr.isti.wnlab.indoornavigation.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

public abstract class DataEmitter<T extends RawData>
        extends AbstractEmitter<T>
        implements StartableStoppable {}
