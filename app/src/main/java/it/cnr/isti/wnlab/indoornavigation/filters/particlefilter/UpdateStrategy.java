package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

/**
 * The update strategy of a Particle Filter.
 * @param <T>
 */
public interface UpdateStrategy<T extends Particle> {

    /**
     * Update particles.
     * @param particles
     */
    void update(Collection<T> particles);
}
