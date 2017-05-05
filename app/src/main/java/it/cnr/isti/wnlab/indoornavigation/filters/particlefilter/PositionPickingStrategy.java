package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;

/**
 * This strategy picks up a position given a collection of particles.
 * @param <T> the type of particle, input of this strategy.
 * @param <P> the type of position, output of this strategy.
 */
public interface PositionPickingStrategy<T extends Particle, P extends XYPosition> {
    P getPosition(Collection<T> particles);
}
