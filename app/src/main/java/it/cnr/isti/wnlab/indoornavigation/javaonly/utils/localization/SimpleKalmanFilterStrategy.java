package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import android.util.Log;

import java.util.ArrayList;
import java.util.Random;

import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.kalmanfilter.KalmanFilter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;
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
            XYPosition startPosition,
            FloorMap chosenFloor,
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
        this.floor = chosenFloor;
        this.kf = new BazookaKalmanFilter();
        this.wiFingMap = wiFingMap;
        this.wiDist = wiDist;
        this.magFingMap = magFingMap;
        this.magDist = magDist;
        this.radius = radius;
        this.r = new Random();

        pdr.register(new Observer<PDR.Result>() {
            @Override
            public void notify(PDR.Result data) {
                XYPosition newPosition = getUpdatedPosition(data);
                notifyObservers(new IndoorPosition(newPosition,floor.getFloor(),System.currentTimeMillis()));
                position = newPosition;
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

        Log.d("KFS", "PDR position is: " + newX + "," + newY);

        // Correct PDR error with Wifi positioning, if possible
        XYPosition fingerprintPosition = getFingerprintPosition();

        Log.d("KFS", "Fingerprint position is: " + fingerprintPosition);

        if(fingerprintPosition != null) {
            // Prediction step (useless for now)
            float[] predictionInput = new float[2];
            predictionInput[0] = 0;
            predictionInput[1] = 0;
            kf.predict(predictionInput);

            Log.d("KFS", "Prediction: " + kf.getStateVector()[0] + "," + kf.getStateVector()[1]);

            // Update step
            float[] updateInput = new float[2];
            updateInput[0] = newX - fingerprintPosition.x;
            updateInput[1] = newY - fingerprintPosition.y;
            kf.update(updateInput);

            Log.d("KFS", "Update: " + kf.getStateVector()[0] + "," + kf.getStateVector()[1]);

            // Pick a random variation in the filtered error
            float[] kfState = kf.getStateVector();
            float errorX = r.nextFloat() * kfState[0];
            float errorY = r.nextFloat() * kfState[1];

            Log.d("KFS", "Errors: " + errorX + "," + errorY);

            newX += errorX;
            newY += errorY;
        }

        Log.d("PFS", "New position: " + position);

        return new XYPosition(newX,newY);
    }

    private XYPosition getFingerprintPosition() {
        // If wifi position is available
        XYPosition wifiPosition = wiDist.findAveragePosition();

        Log.d("FPDEBUG", "null wifiPosition? " + (wifiPosition == null));

        if(wifiPosition != null) {

            Log.d("KFS", "WifiPosition is: " + wifiPosition);

            // Narrow MM positions in Wifi position-centered area
            ArrayList<PositionDistance<XYPosition>> positions = new ArrayList<>();
            for(PositionDistance<XYPosition> p : magDist.getDistances()) {
                Log.d("KFS", "MM position is: " + p + ". Valid? " + GeometryUtils.isPointInCircle(p.position, wifiPosition, radius));
                if (GeometryUtils.isPointInCircle(p.position, wifiPosition, radius))
                    positions.add(p);
            }
            return DistancesMap.findAveragePosition(positions);

        }

        return null;
    }
}
