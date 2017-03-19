package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.PositionFilter2D;

public class IndoorParticleFilter<T extends PositionParticle> extends ParticleFilter<T> implements PositionFilter2D {

    PositionPickingStrategy mPositionPicking;

    public IndoorParticleFilter(
            Collection particles,
            UpdateStrategy updateStep,
            FilteringStrategy filteringStep,
            RegenerationStrategy regenerationStep,
            PositionPickingStrategy positionPicking
    ) {
        super(particles, updateStep, filteringStep, regenerationStep);
        this.mPositionPicking = positionPicking;
    }

    @Override
    public IndoorPosition getPosition(int floor, long timestamp) {
        return new IndoorPosition(get2DPosition(),floor,timestamp);
    }

    @Override
    public XYPosition get2DPosition() {
        super.filter();
        return mPositionPicking.getPosition(mParticles);
    }
}
