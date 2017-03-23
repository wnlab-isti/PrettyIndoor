package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.PositionFilter2D;

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
    public IndoorPosition getPosition(int floor, long timestamp) {
        return new IndoorPosition(position,floor,timestamp);
    }

    @Override
    public XYPosition get2DPosition() {
        return position;
    }
}
