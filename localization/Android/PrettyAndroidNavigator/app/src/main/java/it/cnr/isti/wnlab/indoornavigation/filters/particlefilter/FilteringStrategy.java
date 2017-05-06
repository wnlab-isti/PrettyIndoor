package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

/**
 * Particle filtering step of a Particle Filter.
 * @param <T>
 */
public interface FilteringStrategy<T extends Particle> {

    /**
     * Filter particles.
     * @param particles
     */
    void filter(Collection<T> particles);
}
