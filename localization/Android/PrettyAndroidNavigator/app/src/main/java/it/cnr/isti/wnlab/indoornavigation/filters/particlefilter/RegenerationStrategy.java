package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

public interface RegenerationStrategy {
    /**
     * Make a new collection of particles from another one (i.e. for second generation on).
     * @param particles
     * @return A new collection of particles.
     */
    void regenerate(Collection<Particle> particles);
}
