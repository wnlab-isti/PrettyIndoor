package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.filters.StateEstimationFilter;

/**
 * This is a concrete class for Particle Filters.
 * Actually this is just an empty (enriched) shell: the policies for updating, filtering,
 * regenerating and picking the position as a result must be injected.
 * Note that strategies and the particles collection might be mutable.
 */
public class ParticleFilter<T extends Particle> implements StateEstimationFilter {

    protected Collection<T> particles;
    protected final int beginningParticlesNumber;

    protected UpdateStrategy<T> updateStrategy;
    protected FilteringStrategy<T> filteringStrategy;
    protected ResamplingStrategy<T> resamplingStrategy;

    public ParticleFilter(
            Collection<T> particles,
            UpdateStrategy<T> updateStrategy,
            FilteringStrategy<T> filteringStrategy,
            ResamplingStrategy<T> resamplingStrategy) {
        // Set particles
        this.particles = particles;
        this.beginningParticlesNumber = particles.size();

        // Set strategies
        this.updateStrategy = updateStrategy;
        this.filteringStrategy = filteringStrategy;
        this.resamplingStrategy = resamplingStrategy;
    }

    public void filter() {
        if(updateStrategy != null)
            updateStrategy.update(particles);
        if(filteringStrategy != null)
            filteringStrategy.filter(particles);
        if(resamplingStrategy != null)
            resamplingStrategy.regenerate(particles);
    }

    /*

    THIS CODE SHOULDN'T BE HERE!

    public UpdateStrategy<T> getUpdateStrategy() {
        return updateStrategy;
    }

    public void setUpdateStrategy(UpdateStrategy<T> updateStrategy) {
        this.updateStrategy = updateStrategy;
    }

    public FilteringStrategy<T> getFilteringStrategy() {
        return filteringStrategy;
    }

    public void setFilteringStrategy(FilteringStrategy<T> filteringStrategy) {
        this.filteringStrategy = filteringStrategy;
    }

    public ResamplingStrategy<T> getRegenerationStrategy() {
        return resamplingStrategy;
    }

    public void setRegenerationStrategy(ResamplingStrategy<T> resamplingStrategy) {
        this.resamplingStrategy = resamplingStrategy;
    }*/

}
