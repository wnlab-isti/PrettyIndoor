package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter;

import java.util.Collection;

public interface UpdateStrategy<T extends Particle> {

    /**
     * Update particles.
     * @param particles
     */
    void update(Collection<T> particles);
}
