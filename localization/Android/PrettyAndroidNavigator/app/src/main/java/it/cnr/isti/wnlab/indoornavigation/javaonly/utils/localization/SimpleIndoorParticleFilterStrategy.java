package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import org.apache.commons.math3.distribution.NormalDistribution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import it.cnr.isti.wnlab.indoornavigation.javaonly.AbstractIndoorLocalizationStrategy;
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
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.intertial.pdr.PDR;


public class SimpleIndoorParticleFilterStrategy extends AbstractIndoorLocalizationStrategy {

    // Motion model
    private final static double ANGLE_STANDARD_DEVIATION = Math.PI/2; // N(0,PI/2)
    private final static double SPEED_STANDARD_DEVIATION = 0.15; // N(0,(0.15)^2)
    private NormalDistribution angleDistribution;
    private NormalDistribution speedDistribution;

    // Particle Filter
    private IndoorParticleFilter<PositionParticle> particleFilter;

    // Map
    private FloorMap floorMap;

    // PDR position
    private PDR.Result lastPDRResult;

    // Wifi
    WifiFingerprintMap wifiFing;
    AccessPoints lastWifiList;
    List<FingerprintMap.PositionDistance<XYPosition>> lastWifiDistances;
    static final float WIFI_DISTANCE_THRESHOLD = 10000.f;

    // Magnetic
    MagneticFingerprintMap magFing;
    MagneticField lastMagField;
    List<FingerprintMap.PositionDistance<XYPosition>> lastMagDistances;
    static final float MAGNETIC_DISTANCE_THRESHOLD = 10000.f;

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
            final int particlesNumber,
            // Map
            FloorMap floorMap,
            // PDR
            PDR pdr,
            // Wifi localization
            DataEmitter<AccessPoints> wifi,
            final WifiFingerprintMap wiFing, final int wifiK, final float wifiThreshold,
            // Magnetic mismatch
            DataEmitter<MagneticField> magnetic,
            final MagneticFingerprintMap magFing, final int magK, final float magThreshold
    ) {
        /*
         * Particle Filter
         */
        particleFilter = new IndoorParticleFilter<>(
                /*
                 * Initial particles are all on the initial position.
                 */
                getInitialParticles(initialPosition, particlesNumber),
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
                        regenerate(particles);
                    }
                },
                // Pick a position from particles
                new PositionPickingStrategy<PositionParticle, XYPosition>() {
                    @Override
                    public XYPosition getPosition(Collection<PositionParticle> particles) {
                        return pickPosition(particles);
                    }
                });

        // Initialize motion model
        angleDistribution = new NormalDistribution(0,ANGLE_STANDARD_DEVIATION);
        speedDistribution = new NormalDistribution(0,SPEED_STANDARD_DEVIATION);

        // Floor map
        this.floorMap = floorMap;

        // Register for PDR updates. These trigger the PF.
        pdr.register(new Observer<PDR.Result>() {
            @Override
            public void notify(PDR.Result data) {
                lastPDRResult = data;
                particleFilter.filter();
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

    /**
     * @param initialPosition
     * @param particlesNumber
     * @return A collection of particlesNumber particles on initialPosition.
     */
    private static Collection<PositionParticle> getInitialParticles(XYPosition initialPosition, int particlesNumber) {
        ArrayList<PositionParticle> particles = new ArrayList<>();
        for(int i = 0; i < particlesNumber; i++)
            particles.add(new PositionParticle(initialPosition.x, initialPosition.y, 1.f/particlesNumber));
        return particles;
    }

    /**********************************************************************
     * FIRST STEP: MOVE PARTICLES
     * SECOND STEP: FILTER PARTICLES
     * (these steps are exceptionally together)
     *********************************************************************/

    /**
     * At every step, it is dAngle ~ N(0,PI/2) and dSpeed ~ N(0,(0.15)^2).
     * Second step: filter invalid particles.
     */
    private void updateAndFilter(Collection<PositionParticle> particles) {
        for(PositionParticle p : particles) {
            boolean valid = true;
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
                    !floorMap.isValid(newx,newy) // Map checking
                    || wifiFilterCheck(p) // Wifi fingerprint
                    || magneticFilterCheck(p) // Magnetic fingerprint
            ) { particles.remove(p); valid = false; }

            // If the new position is valid, update particle
            if(valid) {
                p.setX(newx);
                p.setY(newy);
            }
        }
    }

    /**
     * @param p
     * @return true if the particle should survive, false otherwise.
     */
    private boolean wifiFilterCheck(PositionParticle p) {
        // Check if distances are up-to-date
        if(lastWifiDistances == null)
            lastWifiDistances = wifiFing.getDistancedPoints(lastWifiList, WIFI_DISTANCE_THRESHOLD);

        // Call generic function
        return fingerprintFilterCheck(p,lastWifiDistances,WIFI_DISTANCE_THRESHOLD);
    }

    /**
     * @param p
     * @return true if the particle should survive, false otherwise.
     */
    private boolean magneticFilterCheck(PositionParticle p) {
        // Check if distances are up-to-date
        if(lastMagDistances == null)
            lastMagDistances = magFing.getDistancedPoints(lastMagField, MAGNETIC_DISTANCE_THRESHOLD);

        // Call generic function
        return fingerprintFilterCheck(p,lastMagDistances,MAGNETIC_DISTANCE_THRESHOLD);
    }

    /**
     * @param p
     * @param distances
     * @param threshold
     * @return true if the particle should survive, false otherwise.
     */
    private boolean fingerprintFilterCheck(
            PositionParticle p,
            List<FingerprintMap.PositionDistance<XYPosition>> distances,
            float threshold
    ) {
        // Interpolate distance of particle's position.
        // particleDistance = SUM(DISTANCE[i] / R(particle,POINTS[i]) for {i} = pointsInDatabase
        float px = p.getX();
        float py = p.getY();
        float particlePositionDistance = 0.f;
        for(FingerprintMap.PositionDistance<XYPosition> posDis : distances) {
            float dx = posDis.position.x - px;
            float dy = posDis.position.y - py;
            float r = (float) Math.sqrt(dx*dx+dy*dy);
            particlePositionDistance += posDis.distance / r;
        }

        // If random number belongs to [0; positionDistance] kill the particle, else save it.
        // Doing so, the less distanced the particles is, the less the probability of being killed.
        float random = r.nextFloat()*threshold;
        if(random < particlePositionDistance)
            return false;
        else
            return true;
    }

    /**********************************************************************
     * THIRD STEP: Regenerate lost particles
     *********************************************************************/

    // TODO regeneration

    /**********************************************************************
     * FIRST STEP: MOVE PARTICLES
     *********************************************************************/

    /**
     * TODO position picking
     * @param particles
     * @return A position between particles.
     */
    private XYPosition pickPosition(Collection<PositionParticle> particles) {
        return null;
    }
}