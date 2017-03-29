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
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.PositionDistance;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.DistancesMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.GeometryUtils;
import it.cnr.isti.wnlab.indoornavigation.javaonly.pdr.PDR;

public class SimpleKalmanFilterStrategy
        extends AbstractIndoorLocalizationStrategy
        implements Observer<PDR.Result> {

    // Model
    private XYPosition position;
    private FloorMap floor;

    // Kalman Filter
    private KalmanFilter kf;

    // PDR
    private PDR pdr;

    // Wifi
    private DistancesMap<XYPosition, AccessPoints> wiDist;

    // Magnetic Field
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
            final DistancesMap<XYPosition, AccessPoints> wiDist,
            // Magnetic
            DistancesMap<XYPosition, MagneticField> magDist,
            // Wifi filter for MM positions radius
            float radius
    ) {
        this.position = startPosition;
        this.floor = chosenFloor;
        this.pdr = pdr;
        this.kf = new BazookaKalmanFilter();
        this.wiDist = wiDist;
        this.magDist = magDist;
        this.radius = radius;
        this.r = new Random();
    }

    @Override
    public IndoorPosition getCurrentPosition() {
        return new IndoorPosition(position, floor.getFloor(), System.currentTimeMillis());
    }

    private XYPosition getUpdatedPosition(PDR.Result pdrData) {
        // Position with PDR
        float newX = position.x + pdrData.dE;
        float newY = position.y + pdrData.dN;
        Log.d("KFS", "PDR is " + pdrData);

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

//            Log.d("KFS", "Prediction: " + kf.getStateVector()[0] + "," + kf.getStateVector()[1]);

            // Update step
            float[] updateInput = new float[2];
            updateInput[0] = newX - fingerprintPosition.x;
            updateInput[1] = newY - fingerprintPosition.y;
            kf.update(updateInput);

//            Log.d("KFS", "Update: " + kf.getStateVector()[0] + "," + kf.getStateVector()[1]);

            // Pick a random variation in the filtered error
            float[] kfState = kf.getStateVector();
            float errorX = r.nextFloat() * kfState[0];
            float errorY = r.nextFloat() * kfState[1];

            Log.d("KFS", "Errors: " + errorX + "," + errorY);

            // Try correction with Kalman Filtered error
            newX -= errorX;
            newY -= errorY;
        }

        Log.d("KFS", "New position: (" + newX + "," + newY + ") -> " + floor.nearestValid(newX,newY));

        // Choose which is the position to emit
        return floor.nearestValid(newX, newY);
    }

    private XYPosition getFingerprintPosition() {
        // If wifi position is available
        XYPosition wifiPosition = wiDist.findWeightedAveragePosition();

//        Log.d("FPDEBUG", "null wifiPosition? " + (wifiPosition == null));

        if(wifiPosition != null) {

//            Log.d("KFS", "WifiPosition is: " + wifiPosition);

            // Narrow MM positions in Wifi position-centered area
            ArrayList<PositionDistance<XYPosition>> positions = new ArrayList<>();
            for(PositionDistance<XYPosition> p : magDist.getDistances()) {
//                Log.d("KFS", "MM position is: (" + p + "). Valid? " + GeometryUtils.isPointInCircle(p.position, wifiPosition, radius));
                if (GeometryUtils.isPointInCircle(p.position, wifiPosition, radius))
                    positions.add(p);
            }
            return DistancesMap.findWeightedAveragePosition(positions);

        }

        return null;
    }

    @Override
    protected void start() {
        pdr.register(this);
    }

    @Override
    protected void stop() {
        pdr.unregister(this);
    }

    @Override
    public void notify(PDR.Result data) {
        XYPosition newPosition = getUpdatedPosition(data);
        position = newPosition;
        notifyObservers(new IndoorPosition(newPosition,floor.getFloor(),System.currentTimeMillis()));
    }
}
