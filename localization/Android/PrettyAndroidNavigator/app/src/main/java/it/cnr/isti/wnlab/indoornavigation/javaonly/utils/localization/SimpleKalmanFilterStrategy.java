package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import java.util.ArrayList;
import java.util.Random;

import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.kalmanfilter.KalmanFilter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.FingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.MagneticFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.PositionDistance;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.WifiFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.DistancesMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.GeometryUtils;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.pdr.PDR;

public class SimpleKalmanFilterStrategy extends AbstractIndoorLocalizationStrategy {

    // Model
    private XYPosition position;
    private FloorMap floor;

    // Kalman Filter
    private KalmanFilter kf;

    // PDR
    private PDR pdr;

    // Wifi
    private WifiFingerprintMap wiFingMap;
    private DistancesMap<XYPosition, AccessPoints> wiDist;

    // Magnetic Field
    private MagneticFingerprintMap magFingMap;
    private DistancesMap<XYPosition, MagneticField> magDist;

    // Radius
    private float radius;

    // Randomness
    private Random r;

    public SimpleKalmanFilterStrategy(
            IndoorPosition startPosition,
            FloorMap floor,
            // Inertial
            PDR pdr,
            // Wifi
            WifiFingerprintMap wiFingMap,
            final DistancesMap<XYPosition, AccessPoints> wiDist,
            // Magnetic
            MagneticFingerprintMap magFingMap,
            DistancesMap<XYPosition, MagneticField> magDist,
            // Wifi filter for MM positions radius
            float radius
    ) {
        this.position = startPosition;
        this.floor = floor;
        this.kf = new StupidKalmanFilter();
        this.wiFingMap = wiFingMap;
        this.wiDist = wiDist;
        this.magFingMap = magFingMap;
        this.magDist = magDist;
        this.radius = radius;
        this.r = new Random();

        pdr.register(new Observer<PDR.Result>() {
            @Override
            public void notify(PDR.Result data) {
                position = getUpdatedPosition(data);
            }
        });
    }

    @Override
    public IndoorPosition getCurrentPosition() {
        return new IndoorPosition(position, floor.getFloor(), System.currentTimeMillis());
    }

    private XYPosition getUpdatedPosition(PDR.Result pdrData) {
        // Position with PDR
        float newX = position.x + pdrData.dE;
        float newY = position.y + pdrData.dN;

        // Correct PDR error with Wifi positioning, if possible
        XYPosition fingerprintPosition = getFingerprintPosition();
        if(fingerprintPosition != null) {
            // Prediction step (useless for now)
            float[] predictionInput = new float[2];
            predictionInput[0] = 0;
            predictionInput[1] = 0;
            kf.predict(predictionInput);

            // Update step
            float[] updateInput = new float[2];
            updateInput[0] = newX - fingerprintPosition.x;
            updateInput[1] = newY - fingerprintPosition.y;
            kf.update(updateInput);

            // Pick a random variation in the filtered error
            float[] kfState = kf.getStateVector();
            float errorX = r.nextFloat() * kfState[0];
            float errorY = r.nextFloat() * kfState[1];
            newX += errorX;
            newY += errorY;
        }

        return new XYPosition(newX,newY);
    }

    private XYPosition getFingerprintPosition() {
        // If wifi position is available
        XYPosition wifiPosition = wiDist.findAveragePosition();
        if(wifiPosition != null) {

            // Narrow MM positions in Wifi position-centered area
            ArrayList<PositionDistance<XYPosition>> positions = new ArrayList<>();
            for(PositionDistance<XYPosition> p : magDist.getDistances())
                if(GeometryUtils.isPointInCircle(p.position, wifiPosition, radius))
                    positions.add(p);
            return DistancesMap.findAveragePosition(positions);

        }

        return null;
    }
}
