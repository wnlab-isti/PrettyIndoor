package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import android.util.Log;

import org.apache.commons.math3.distribution.NormalDistribution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import it.cnr.isti.wnlab.indoornavigation.androidapp.localization.Constants;
import it.cnr.isti.wnlab.indoornavigation.javaonly.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.IndoorParticleFilter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.PositionParticle;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.PositionPickingStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.RegenerationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.UpdateStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.FingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.MagneticFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.WifiFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.pdr.PDR;


public class SimpleIndoorParticleFilterStrategy extends AbstractIndoorLocalizationStrategy {

    // Motion model
    private final static double ANGLE_STANDARD_DEVIATION = Math.PI/2; // N(0,PI/2)
    private final static double SPEED_STANDARD_DEVIATION = 0.15; // N(0,(0.15)^2)
    private final NormalDistribution angleDistribution;
    private final NormalDistribution speedDistribution;

    // Particle Filter
    private IndoorParticleFilter particleFilter;
    private final int particlesNumber;

    // Map
    private FloorMap floorMap;

    // PDR position
    private PDR.Result lastPDRResult;

    // Wifi
    private WifiFingerprintMap wifiFing;
    private AccessPoints lastWifiList;
    private List<FingerprintMap.PositionDistance<XYPosition>> lastWifiDistances;
    private float wifiDistanceMaxLimit;

    // Magnetic
    private MagneticFingerprintMap magFing;
    private MagneticField lastMagField;
    private List<FingerprintMap.PositionDistance<XYPosition>> lastMagDistances;
    private float magneticDistanceMaxLimit;

    // Random numbers
    Random r;

    /**
     * Strategy using a ParticleFilter with initial position.
     * @param initialPosition
     * @param particlesNumber
     */
    public SimpleIndoorParticleFilterStrategy(
            // Initial position and particles number
            XYPosition initialPosition,
            int particlesNumber,
            // Map
            FloorMap floorMap,
            // PDR
            PDR pdr,
            // Wifi localization
            DataEmitter<AccessPoints> wifi,
            WifiFingerprintMap wiFing,
            // Magnetic mismatch
            DataEmitter<MagneticField> magnetic,
            MagneticFingerprintMap magFing
    ) {
        /*
         * Motion model
         */

        this.angleDistribution = new NormalDistribution(0,ANGLE_STANDARD_DEVIATION);
        this.speedDistribution = new NormalDistribution(0,SPEED_STANDARD_DEVIATION);
        this.floorMap = floorMap;

        /*
         * Particle Filter
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
        pdr.register(new Observer<PDR.Result>() {
            @Override
            public void notify(PDR.Result data) {
                Log.d("PFS", "PDR arrived " + data);
                lastPDRResult = data;
                particleFilter.filter();
                notifyObservers(particleFilter.getPosition(-1,System.currentTimeMillis()));
            }
        });

        /*
         * Wifi
         */

        // Wifi fingerprint map
        this.wifiFing = wiFing;
        // Wifi last measurement
        lastWifiList = null;
        // Row distances per position from last Wifi scansion
        lastWifiDistances = null; // lazy policy
        // Wifi AP list observer
        wifi.register(new Observer<AccessPoints>() {
            @Override
            public void notify(AccessPoints data) {
                lastWifiList = data;
                lastWifiDistances = null;
            }
        });

        /*
         * Magnetic
         */

        // Magnetic fingerprint map
        this.magFing = magFing;
        // Last magnetic field measurement
        lastMagField = null;
        // Row distances per position from last magnetic field measurement
        lastMagDistances = null; // lazy policy
        magnetic.register(new Observer<MagneticField>() {
            @Override
            public void notify(MagneticField data) {
                lastMagField = data;
                lastMagDistances = null;
            }
        });

        /*
         * Randomness
         */
        r = new Random();
    }

    @Override
    public IndoorPosition getCurrentPosition() {
        return particleFilter.getPosition(Constants.INITIAL_FLOOR,System.currentTimeMillis());
    }

    /**********************************************************************
     * FIRST STEP: Move particles
     * SECOND STEP: Filter particles
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
            float dx = (lastPDRResult.dE + (float) speedDistribution.sample()) *
                    ((float) Math.cos(lastPDRResult.heading + angleDistribution.sample()));
            float newx = p.getX() + dx;
            // Update y
            float dy = (lastPDRResult.dN + (float) speedDistribution.sample()) *
                    ((float) Math.cos(lastPDRResult.heading + angleDistribution.sample()));
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
        int k = 3;
        // Check if distances are up-to-date
        if(lastWifiDistances == null) {
            lastWifiDistances = wifiFing.findNearestK(lastWifiList, k, Float.MAX_VALUE);
            wifiDistanceMaxLimit = getMaxLimit(lastWifiDistances);
            Log.d("PFS", "Wifi distances updated (" + lastWifiDistances.size() + ").");
        }

        // Call generic function
        Log.d("PFS", "Wifi check");
        return fingerprintFilterCheck(p,lastWifiDistances, wifiDistanceMaxLimit);
    }

    /**
     * @param p
     * @return true if the particle should survive, false otherwise.
     */
    private boolean magneticFilterCheck(PositionParticle p) {
        int k = 3;
        // Check if distances are up-to-date
        if(lastMagDistances == null) {
            lastMagDistances = magFing.findNearestK(lastMagField, k, Float.MAX_VALUE);
            magneticDistanceMaxLimit = getMaxLimit(lastMagDistances);
            Log.d("PFS", "Magnetic distances updated (" + lastMagDistances.size() + ").");
        }

        // Call generic function
        Log.d("PFS", "Magnetic check");
        return fingerprintFilterCheck(p,lastMagDistances, magneticDistanceMaxLimit);
    }

    /**
     * When filtering by fingerprints, the particle dies when the random number between 0 and N
     * is less than particle's interpolated distance.
     * This function decides N.
     * @param pds
     * @return
     */
    public float getMaxLimit(List<FingerprintMap.PositionDistance<XYPosition>> pds) {
        // Find average between distances and minimum distance
        float dSum = 0.f;
        float minDistance = Float.MAX_VALUE;
        float maxDistance = 0.f;
        for(FingerprintMap.PositionDistance<XYPosition> pd : pds) {
            dSum += pd.distance;
            if(pd.distance < minDistance)
                minDistance = pd.distance;
            if(pd.distance > maxDistance)
                maxDistance = pd.distance;
        }
        float avgDistance = dSum / pds.size();

        // Set as max boundary: AVG(distances)+AVG(AVG(DISTANCES),MIN_DISTANCE))
        float boundary = maxDistance;

        return boundary;
    }

    /**
     * @param p
     * @param distances
     * @return true if the particle should survive, false otherwise.
     */
    private boolean fingerprintFilterCheck(
            PositionParticle p,
            List<FingerprintMap.PositionDistance<XYPosition>> distances,
            float maxLimit
    ) {
        // Interpolate distance of particle's position.
        // particleDistance = SUM(DISTANCE[Pi] / R(particle,POINTS[Pi]) for {i} = pointsInDatabase
        float px = p.getX();
        float py = p.getY();
        float particlePositionDistance = 0.f;
        float rSum = 0.f;
        for(FingerprintMap.PositionDistance<XYPosition> posDis : distances) {
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
        if(random < particlePositionDistance)
            return false;
        else
            return true;
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

        /*
        // Set weight of M survived particles as 1/(M+1)
        int survivedParticlesN = particles.size();
        float survivedParticlesWeight = 1.f/(survivedParticlesN+1);
        for(PositionParticle p : particles)
            p.setWeight(survivedParticlesWeight);

        // Duplicate old particles in order to create new N = originalNumber-M particles.
        // New particles' weights are 1/(M+1) * 1/N
        int newParticlesN = particlesNumber - survivedParticlesN;
        float newParticlesWeight = 1.f/( (survivedParticlesN+1)*newParticlesN );
        PositionParticle[] particlesArray = particles.toArray(new PositionParticle[survivedParticlesN]);
        for(int i = 0; i < newParticlesN; i++) {
            PositionParticle newParticle = particlesArray[i%survivedParticlesN].clone(newParticlesWeight);
            particles.add(newParticle);
        }*/
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
}