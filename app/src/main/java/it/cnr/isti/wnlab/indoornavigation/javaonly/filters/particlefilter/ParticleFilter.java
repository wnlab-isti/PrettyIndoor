package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.Filter;

public class ParticleFilter<T extends Particle> implements Filter {

    protected Collection<T> mParticles;
    private UpdateStrategy<T> mUpdateStep;
    private FilteringStrategy<T> mFilteringStep;
    private RegenerationStrategy<T> mRegenerationStep;

    public ParticleFilter(
            Collection<T> particles,
            UpdateStrategy<T> updateStep,
            FilteringStrategy<T> filteringStep,
            RegenerationStrategy<T> regenerationStep) {
        mParticles = particles;
        mUpdateStep = updateStep;
        mFilteringStep = filteringStep;
        mRegenerationStep = regenerationStep;
    }

    public Collection<T> filter() {
        mUpdateStep.update(mParticles);
        mFilteringStep.filter(mParticles);
        mRegenerationStep.regenerate(mParticles);
        return mParticles;
    }

}
