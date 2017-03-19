package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter;

import java.util.Collection;

public interface FilteringStrategy<T extends Particle> {

    /**
     * Filter particles.
     * @param particles
     */
    void filter(Collection<T> particles);
}
