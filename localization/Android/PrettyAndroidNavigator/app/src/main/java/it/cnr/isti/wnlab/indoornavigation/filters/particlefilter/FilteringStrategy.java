package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

public interface FilteringStrategy {

    /**
     * Filter particles.
     * @param particles
     */
    void filter(Collection<Particle> particles);
}
