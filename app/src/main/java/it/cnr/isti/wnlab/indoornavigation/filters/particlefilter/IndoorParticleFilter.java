package it.cnr.isti.wnlab.indoornavigation.filters.particlefilter;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.filters.PositionFilter2D;

/**
 * This is a concrete class for Particle Filters for Indoor 2D Localization.
 * Actually this is just an empty (enriched) shell: the policies for updating, filtering,
 * regenerating and picking the position as a result must be injected.
 */
public class IndoorParticleFilter extends ParticleFilter<PositionParticle> implements PositionFilter2D {

    protected PositionPickingStrategy mPositionPicking;
    protected XYPosition position;

    public IndoorParticleFilter(
            XYPosition initialPosition,
            Collection particles,
            UpdateStrategy updateStep,
            FilteringStrategy filteringStep,
            RegenerationStrategy regenerationStep,
            PositionPickingStrategy<PositionParticle,XYPosition> positionPicking
    ) {
        super(particles, updateStep, filteringStep, regenerationStep);
        this.mPositionPicking = positionPicking;
        this.position = initialPosition;
    }

    @Override
    public void filter() {
        super.filter();
        position = mPositionPicking.getPosition(particles);
    }

    @Override
    public XYPosition get2DPosition() {
        return position;
    }
}
