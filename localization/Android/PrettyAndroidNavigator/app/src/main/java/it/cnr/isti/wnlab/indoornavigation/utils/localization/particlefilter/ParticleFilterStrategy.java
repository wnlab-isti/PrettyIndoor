package it.cnr.isti.wnlab.indoornavigation.utils.localization.particlefilter;

import android.util.Log;

import org.apache.commons.math3.distribution.NormalDistribution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import it.cnr.isti.wnlab.indoornavigation.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.filters.particlefilter.IndoorParticleFilter;
import it.cnr.isti.wnlab.indoornavigation.filters.particlefilter.PositionParticle;
import it.cnr.isti.wnlab.indoornavigation.filters.particlefilter.PositionPickingStrategy;
import it.cnr.isti.wnlab.indoornavigation.filters.particlefilter.RegenerationStrategy;
import it.cnr.isti.wnlab.indoornavigation.filters.particlefilter.UpdateStrategy;
import it.cnr.isti.wnlab.indoornavigation.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.MagneticFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.PositionDistance;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.WifiFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.DistancesMap;
import it.cnr.isti.wnlab.indoornavigation.pdr.PDR;

/**
 * A localization strategy that uses the Particle Filter.
 */
public class ParticleFilterStrategy
        extends AbstractIndoorLocalizationStrategy
        implements Observer<PDR.Result> {

    // Motion model
    private final static double ANGLE_STANDARD_DEVIATION = Math.PI/2; // N(0,PI/2)
    private final static double SPEED_STANDARD_DEVIATION = 0.15; // N(0,(0.15)^2)
    private final NormalDistribution angleDistribution;
    private final NormalDistribution speedDistribution;
    private float stepLength;

    // Particle StateEstimationFilter
    private IndoorParticleFilter particleFilter;
    private final int particlesNumber;

    // Map
    private FloorMap floorMap;

    // PDR and inertial
    private PDR pdr;
    private PDR.Result lastPDRResult;

    // Wifi
    private WifiFingerprintMap wifiFing;
    private float wifiDistanceMaxLimit;
    private DistancesMap<XYPosition,AccessPoints> lastWifiDistances;

    // Magnetic
    private MagneticFingerprintMap magFing;
    private float magneticDistanceMaxLimit;
    private DistancesMap<XYPosition,MagneticField> lastMagDistances;

    // Random numbers
    private Random r;

    /**
     * Strategy using a ParticleFilter with initial position.
     * @param initialPosition
     * @param particlesNumber
     */
    public ParticleFilterStrategy(
            // Initial position and particles number
            XYPosition initialPosition,
            int particlesNumber,
            // Map
            final FloorMap floorMap,
            // Featured step length
            float stepLength,
            // PDR
            PDR pdr,
            // Wifi localization
            WifiFingerprintMap wiFing,
            DistancesMap<XYPosition, AccessPoints> wifiDist,
            // Magnetic mismatch
            MagneticFingerprintMap magFing,
            DistancesMap<XYPosition, MagneticField> magDist
    ) {
        /*
         * Motion model
         */

        this.angleDistribution = new NormalDistribution(0,ANGLE_STANDARD_DEVIATION);
        this.speedDistribution = new NormalDistribution(0,SPEED_STANDARD_DEVIATION);
        this.floorMap = floorMap;
        this.stepLength = stepLength;

        /*
         * Particle StateEstimationFilter
         */

        // Initialize PF
        this.particlesNumber = particlesNumber;
        particleFilter = new IndoorParticleFilter(
                // Initial position
                initialPosition,
                // Create particles
                PositionParticle.createParticles(initialPosition, particlesNumber),
                // Move particles
                new UpdateStrategy<PositionParticle>() {
                    @Override
                    public void update(Collection<PositionParticle> particles) {
                        updateAndFilter(particles);
                    }
                },
                // Filtering is in previous function (to save an iteration over all particles)
                null,
                // Regenerate particles
                new RegenerationStrategy<PositionParticle>() {
                    @Override
                    public void regenerate(Collection<PositionParticle> particles) {
                        regenerateParticles(particles);
                    }
                },
                // Pick a position from particles
                new PositionPickingStrategy<PositionParticle, XYPosition>() {
                    @Override
                    public XYPosition getPosition(Collection<PositionParticle> particles) {
                        return pickWeightAvgPosition(particles);
                    }
                });

        /*
         * PDR
         */

        // Register for PDR updates. These trigger the PF.
        this.pdr = pdr;

        /*
         * Wifi
         */

        // Wifi fingerprint and distances map
        this.wifiFing = wiFing;
        this.lastWifiDistances = wifiDist;

        /*
         * Magnetic
         */

        // Magnetic fingerprint and distances map
        this.magFing = magFing;
        this.lastMagDistances = magDist;

        /*
         * Randomness
         */
        r = new Random();
    }

    @Override
    public IndoorPosition getCurrentPosition() {
        return new IndoorPosition(particleFilter.get2DPosition(),floorMap.getFloor(),System.currentTimeMillis());
    }

    /**********************************************************************
     * FIRST STEP: Move particles
     * SECOND STEP: StateEstimationFilter particles
     * (these steps are exceptionally together)
     *********************************************************************/

    /**
     * At every step, it is dAngle ~ N(0,PI/2) and dSpeed ~ N(0,(0.15)^2).
     * Second step: filter invalid particles.
     */
    private void updateAndFilter(Collection<PositionParticle> particles) {
        Log.d("PF","Update step");

        // Needed for avoiding ConcurrentModificationException during iteration
        ArrayList<PositionParticle> obsoleteParticles = new ArrayList<>();

        // Check for particles to eliminate
        for(PositionParticle p : particles) {
            Log.d("PF", "Updating " + p);

            // Update x
            float dx = (stepLength + (float) speedDistribution.sample()) *
                    ((float) Math.cos(lastPDRResult.heading + angleDistribution.sample()));
            Log.d("PFS", "dE is " + lastPDRResult.dE + ", dx is " + dx);
            float newx = p.getX() + dx;
            // Update y
            float dy = -(stepLength + (float) speedDistribution.sample()) *
                    ((float) Math.sin(lastPDRResult.heading + angleDistribution.sample()));
            Log.d("PFS", "dN is " + lastPDRResult.dN + ", dy is " + dy);
            float newy = p.getY() + dy;

            // Let's break the schema: filter here
            if(
                    floorMap.isValid(newx,newy) // Map checking
                    && wifiFilterCheck(p) // Wifi fingerprint
                    && magneticFilterCheck(p) // Magnetic fingerprint
            ) {
                p.setX(newx);
                p.setY(newy);
                Log.d("PF", "Survived");
            } else {
                obsoleteParticles.add(p);
                Log.d("PF", "Killed");
            }
        }

        // Remove obsolete particles
        particles.removeAll(obsoleteParticles);
        Log.d("PF", obsoleteParticles.size() + " particles removed");
    }

    /**
     * @param p
     * @return true if the particle should survive, false otherwise.
     */
    private boolean wifiFilterCheck(PositionParticle p) {
        List<PositionDistance<XYPosition>> distances = lastWifiDistances.getDistances();
        wifiDistanceMaxLimit = getMaxLimit(distances);

        // Call generic function
        Log.d("PFS", "Magnetic check");
        return fingerprintFilterCheck(p, distances, wifiDistanceMaxLimit);
    }

    /**
     * @param p
     * @return true if the particle should survive, false otherwise.
     */
    private boolean magneticFilterCheck(PositionParticle p) {
        List<PositionDistance<XYPosition>> distances = lastMagDistances.getDistances();
        magneticDistanceMaxLimit = getMaxLimit(distances);

        // Call generic function
        Log.d("PFS", "Magnetic check");
        return fingerprintFilterCheck(p, distances, magneticDistanceMaxLimit);
    }

    /**
     * When filtering by fingerprints, the particle dies when the random number between 0 and N
     * is less than particle's interpolated distance.
     * This function decides N.
     * @param pds
     * @return
     */
    public float getMaxLimit(List<PositionDistance<XYPosition>> pds) {
        // Find average between distances and minimum distance
        /*float dSum = 0.f;*/
        /*float minDistance = Float.MAX_VALUE;*/
        float maxDistance = 0.f;
        for(PositionDistance<XYPosition> pd : pds) {
            /*dSum += pd.distance;*/
            /*if(pd.distance < minDistance)
                minDistance = pd.distance;*/
            if(pd.distance > maxDistance)
                maxDistance = pd.distance;
        }
        /*float avgDistance = dSum / pds.size();*/

        return maxDistance;
    }

    /**
     * @param p
     * @param distances
     * @return true if the particle should survive, false otherwise.
     */
    private boolean fingerprintFilterCheck(
            PositionParticle p,
            List<PositionDistance<XYPosition>> distances,
            float maxLimit
    ) {
        // Interpolate distance of particle's position.
        // particleDistance = SUM(DISTANCE[Pi] / R(particle,POINTS[Pi]) for {i} = pointsInDatabase
        float px = p.getX();
        float py = p.getY();
        float particlePositionDistance = 0.f;
        float rSum = 0.f;
        for(PositionDistance<XYPosition> posDis : distances) {
            float dx = posDis.position.x - px;
            float dy = posDis.position.y - py;
            float r = (float) Math.sqrt(dx*dx+dy*dy);
            particlePositionDistance += ( posDis.distance * (1/r) );
            rSum += r;
        }
        particlePositionDistance /= rSum;

        // If random number belongs to [0; positionDistance] kill the particle, else save it.
        // Doing so, the less distanced the particles is, the less the probability of being killed.
        float random = r.nextFloat()*(maxLimit);
        Log.d("PFS", "Random is " + random + ", distance is " + particlePositionDistance + ", threshold: " + maxLimit + ", distances: " + distances.size());
        return random >= particlePositionDistance;
    }

    /**********************************************************************
     * THIRD STEP: Regenerate lost particles
     *********************************************************************/

    private void regenerateParticles(Collection<PositionParticle> particles) {

        Log.d("PF", "Regeneration step");

        // Assure that there's at least one particle
        if(particles.isEmpty()) {
            XYPosition position = particleFilter.get2DPosition();
            float newX = position.x + lastPDRResult.dE;
            float newY = position.y + lastPDRResult.dN;
            particles.add(new PositionParticle(newX, newY, 1.f / particlesNumber));
        }

        // Duplicate old particles in order to create new N = originalNumber-M particles.
        // All the particles have ALWAYS THE SAME WEIGHT.
        int survivedParticlesN = particles.size();
        int newParticlesN = particlesNumber - particles.size();
        float newParticlesWeight = 1.f/particlesNumber;
        PositionParticle[] particlesArray = particles.toArray(new PositionParticle[survivedParticlesN]);
        for(int i = 0; i < newParticlesN; i++) {
            Log.d("PF", "Duplicating " + particlesArray[i%survivedParticlesN]);
            // Iterate on first survived particles and add new ones.
            PositionParticle newParticle = particlesArray[i%survivedParticlesN].clone(newParticlesWeight);
            particles.add(newParticle);
        }

    }

    /**********************************************************************
     * Pick a position to signal from particles.
     *********************************************************************/

    /**
     * @param particles
     * @return A position between particles in respecting particles weights.
     */
    private XYPosition pickWeightAvgPosition(Collection<PositionParticle> particles) {
        float avgX = 0.f;
        float avgY = 0.f;
        for(PositionParticle p : particles) {
            float weight = p.getWeight();
            avgX += weight * p.getX();
            avgY += weight * p.getY();
        }
        return new XYPosition(avgX,avgY);
    }

    @Override
    protected void startEmission() {
        pdr.register(this);
    }

    @Override
    protected void stopEmission() {
        pdr.unregister(this);
    }

    @Override
    public void notify(PDR.Result data) {
        lastPDRResult = data;
        particleFilter.filter();
        notifyObservers(new IndoorPosition(particleFilter.get2DPosition(),floorMap.getFloor(),System.currentTimeMillis()));
    }
}