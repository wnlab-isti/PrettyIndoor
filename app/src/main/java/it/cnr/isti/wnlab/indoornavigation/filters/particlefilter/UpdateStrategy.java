package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

public interface UpdateStrategy {

    /**
     * Update particles.
     * @param particles
     */
    void update(Collection<Particle> particles);
}
