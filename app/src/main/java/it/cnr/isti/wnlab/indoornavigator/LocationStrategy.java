package it.cnr.isti.wnlab.indoornavigator;

import it.cnr.isti.wnlab.indoornavigator.emitters.AbstractEmitter;

/**
 * Abstract strategy for 2,5D location (x,y,floor).
 * Notifies a position to its observers.
 */
public abstract class LocationStrategy extends AbstractEmitter<IndoorPosition> {}
