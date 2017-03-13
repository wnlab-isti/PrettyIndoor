package it.cnr.isti.wnlab.indoornavigation;

import it.cnr.isti.wnlab.indoornavigation.observer.AbstractEmitter;

/**
 * Abstract strategy for 2,5D location (x,y,floor).
 * Notifies a position to its observers.
 */
public abstract class AbstractLocationStrategy extends AbstractEmitter<IndoorPosition> implements LocationStrategy {}
