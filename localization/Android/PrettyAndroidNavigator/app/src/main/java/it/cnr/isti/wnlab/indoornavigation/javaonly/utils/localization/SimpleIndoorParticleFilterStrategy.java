package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import java.util.ArrayList;
import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.FilteringStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.IndoorParticleFilter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.PositionParticle;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.PositionPickingStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.RegenerationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.UpdateStrategy;


public class SimpleIndoorParticleFilterStrategy extends AbstractIndoorLocalizationStrategy {

    private IndoorParticleFilter<PositionParticle> particleFilter;

    /**
     * Strategy using a ParticleFilter with initial position.
     * @param initialPosition
     * @param particlesNumber
     */
    public SimpleIndoorParticleFilterStrategy(
        XYPosition initialPosition,
        int particlesNumber
    ) {
        particleFilter = new IndoorParticleFilter<>(
                /*
                 * Initial particles are all on the initial position.
                 */
                getInitialParticles(initialPosition, particlesNumber),
                // Private classes below
                new SimpleUpdateStrategy(), new SimpleFilterStrategy(), new SimpleRegenerationStrategy(), new SimplePickingStrategy());
    }

    /**
     * Strategy using a ParticleFilter without initial position.
     * @param particlesNumber
     * @param minx
     * @param miny
     * @param maxx
     * @param maxy
     */
    public SimpleIndoorParticleFilterStrategy(
            int particlesNumber, float minx, float miny, float maxx, float maxy) {

        particleFilter = new IndoorParticleFilter<>(
                /*
                 * Initial particles are gaussian distributed.
                 */
                PositionParticle.getGaussianCollection(particlesNumber, minx, miny, maxx, maxy),
                // Private classes below
                new SimpleUpdateStrategy(), new SimpleFilterStrategy(), new SimpleRegenerationStrategy(), new SimplePickingStrategy());
    }

    /**
     * @param initialPosition
     * @param particlesNumber
     * @return A collection of particlesNumber particles on initialPosition.
     */
    private Collection<PositionParticle> getInitialParticles(XYPosition initialPosition, int particlesNumber) {
        ArrayList<PositionParticle> particles = new ArrayList<>();
        for(int i = 0; i < particlesNumber; i++)
            particles.add(new PositionParticle(initialPosition.x, initialPosition.y, 1.f/particlesNumber));
        return particles;
    }

    private class SimpleUpdateStrategy implements UpdateStrategy<PositionParticle> {
        @Override
        public void update(Collection<PositionParticle> particles) {

        }
    }

    private class SimpleFilterStrategy implements FilteringStrategy<PositionParticle> {
        @Override
        public void filter(Collection<PositionParticle> particles) {

        }
    }

    private class SimpleRegenerationStrategy implements RegenerationStrategy<PositionParticle> {
        @Override
        public void regenerate(Collection<PositionParticle> particles) {

        }
    }

    private class SimplePickingStrategy implements PositionPickingStrategy<PositionParticle,XYPosition> {
        @Override
        public XYPosition getPosition(Collection<PositionParticle> particles) {
            return null;
        }
    }

}
