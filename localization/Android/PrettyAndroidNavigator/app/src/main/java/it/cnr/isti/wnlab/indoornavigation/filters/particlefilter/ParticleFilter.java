package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

public abstract class ParticleFilter {

    private Collection<Particle> mParticles;
    private UpdateStrategy mUpdateStep;
    private FilteringStrategy mFilteringStep;
    private RegenerationStrategy mRegenerationStep;

    public ParticleFilter(
            Collection<Particle> particles,
            UpdateStrategy updateStep,
            FilteringStrategy filteringStep,
            RegenerationStrategy regenerationStep) {
        mParticles = particles;
        mUpdateStep = updateStep;
        mFilteringStep = filteringStep;
        mRegenerationStep = regenerationStep;
    }

    public void filter() {
        mUpdateStep.update(mParticles);
        mFilteringStep.filter(mParticles);
        mRegenerationStep.regenerate(mParticles);
    }

}
