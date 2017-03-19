package it.cnr.isti.wnlab.indoornavigation.javaonly;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.AbstractEmitter;

/**
 * Abstract strategy for 2,5D location (x,y,floor).
 * Notifies a position to its observers.
 */
public abstract class AbstractIndoorLocalizationStrategy extends AbstractEmitter<IndoorPosition> implements IndoorLocalizationStrategy {}