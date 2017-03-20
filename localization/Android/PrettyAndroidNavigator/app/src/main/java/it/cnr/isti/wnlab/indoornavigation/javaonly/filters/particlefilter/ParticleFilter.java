package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.Filter;

public class ParticleFilter<T extends Particle> implements Filter {

    protected Collection<T> particles;
    protected final int originalParticlesNumber;

    private UpdateStrategy<T> updateStrategy;
    private FilteringStrategy<T> filteringStrategy;
    private RegenerationStrategy<T> regenerationStrategy;

    public ParticleFilter(
            Collection<T> particles,
            UpdateStrategy<T> updateStrategy,
            FilteringStrategy<T> filteringStrategy,
            RegenerationStrategy<T> regenerationStrategy) {
        // Set particles
        this.particles = particles;
        this.originalParticlesNumber = particles.size();

        // Set strategies
        this.updateStrategy = updateStrategy;
        this.filteringStrategy = filteringStrategy;
        this.regenerationStrategy = regenerationStrategy;
    }

    public Collection<T> filter() {
        if(updateStrategy != null)
            updateStrategy.update(particles);
        if(filteringStrategy != null)
            filteringStrategy.filter(particles);
        if(regenerationStrategy != null)
            regenerationStrategy.regenerate(particles);
        return particles;
    }

    /*
     * Strategies are mutable (unlike particles)
     */

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

    public RegenerationStrategy<T> getRegenerationStrategy() {
        return regenerationStrategy;
    }

    public void setRegenerationStrategy(RegenerationStrategy<T> regenerationStrategy) {
        this.regenerationStrategy = regenerationStrategy;
    }

}
