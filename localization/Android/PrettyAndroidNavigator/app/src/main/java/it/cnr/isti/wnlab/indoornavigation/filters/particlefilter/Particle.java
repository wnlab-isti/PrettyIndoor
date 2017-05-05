package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

/**
 * A generic Particle for Particle StateEstimationFilter.
 */
public interface Particle {
    float getWeight();
    void setWeight(float weight);
}
