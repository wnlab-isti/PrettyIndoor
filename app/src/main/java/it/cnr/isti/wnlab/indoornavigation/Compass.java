package it.cnr.isti.wnlab.indoornavigation;

import it.cnr.isti.wnlab.indoornavigation.observer.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigation.types.Heading;

/**
 * Compass is supposed to be an emitter of Heading objects. Extend this class for making your own
 * compass implementation.
 */
public abstract class Compass extends AbstractEmitter<Heading> {}
