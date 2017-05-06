package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

/**
 * Particle Filter's particle.
 */
public interface Particle {
    float getWeight();
    void setWeight(float weight);
}
