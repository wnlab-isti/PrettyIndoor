package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

/**
 * The regeneration policy of a Particle StateEstimationFilter.
 * @param <T>
 */
public interface ResamplingStrategy<T extends Particle> {
    /**
     * Make a new collection of particles from another one (i.e. for second generation on).
     * @param particles
     * @return A new collection of particles.
     */
    void regenerate(Collection<T> particles);
}
