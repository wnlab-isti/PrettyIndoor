package it.cnr.isti.wnlab.indoornavigation.android.service;

import android.content.Context;
import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.widget.Toast;

import java.io.File;
import java.lang.ref.WeakReference;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.android.compass.RelativeCompass;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.InvalidSensorException;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.MagnetometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.WifiScanner;
import it.cnr.isti.wnlab.indoornavigation.android.stepdetection.FasterStepDetector;
import it.cnr.isti.wnlab.indoornavigation.javaonly.Compass;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorNavigator;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.StepDetector;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.pdr.PDR;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.DistancesMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.MagneticFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.PositionDistance;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.WifiFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.fingerprint.FingerprintStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.kalmanfilter.KalmanFilterStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.pdr.FixedStepPDR;

class KFIndoorNavigator implements IndoorNavigator {

    /*
     * Android Context
     */

    private WeakReference<Context> context;

    /*
     * Localization strategy
     */

    private IndoorLocalizationStrategy strategy;
    private XYPosition position;
    private FloorMap floorMap;

    private Observer<IndoorPosition> observer;

    private boolean active;

    /*
     * Handlers
     */

    // Sensor handlers
    private AccelerometerHandler ah;
    private GyroscopeHandler gh;
    private MagnetometerHandler mh;

    // Wifi
    private WifiScanner wifi;

    /*
     * PDR
     */

    private Compass compass;
    private StepDetector stepDetector;
    private PDR pdr;

    /*
     * Fingerprints
     */

    private WifiFingerprintMap wiFing;
    private DistancesMap<XYPosition, AccessPoints> wifiDist;

    private MagneticFingerprintMap magFing;
    private DistancesMap<XYPosition, MagneticField> magDist;

    // Wifi fingerprint localization. NOT USED EITHER IN PF OR KF
    private FingerprintStrategy<AccessPoints> wifiLocalization;
    // Magnetic fingerprint localization. NOT USED EITHER IN PF OR KF
    private FingerprintStrategy<MagneticField> magneticLocalization;

    /**********************************************
     * Getters and setters
     **********************************************/

    public void setStart(XYPosition position, FloorMap floorMap) {
        setStartPosition(position);
        setStartFloor(floorMap);
    }

    // Start Position

    public void setStartPosition(XYPosition position) {
        this.position = position;
    }

    public XYPosition getStartPosition() {
        return this.position;
    }

    // Start Floor

    public void setStartFloor(FloorMap floorMap) {
        this.floorMap = floorMap;
    }

    public FloorMap getStartFloor() {
        return this.floorMap;
    }

    // Observer for position updates

    public void setObserver(Observer<IndoorPosition> observer) {
        this.observer = observer;
    }

    public Observer<IndoorPosition> getObserver() {
        return observer;
    }

    /**********************************************
     * Initialization
     **********************************************/

    public KFIndoorNavigator(Context context) {
        this.context = new WeakReference<>(context);
        this.active = false;
        initializeHandlers();
    }

    /**
     * Initializes sensor and wifi handlers. If they are not available, the related ToggleButton is
     * deactivated.
     */
    private void initializeHandlers() {
        Context context = this.context.get();
        if(context == null)
            return;

        // Initialize sensors and handlers
        SensorManager manager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);

        // Accelerometer
        try {
            ah = new AccelerometerHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        } catch(InvalidSensorException e) {
            Toast.makeText(context,context.getString(R.string.acc_not_available),Toast.LENGTH_SHORT).show();
            ah = null;
        }
        try {
            gh = new GyroscopeHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        } catch(InvalidSensorException e) {
            Toast.makeText(context,context.getString(R.string.gyro_not_available),Toast.LENGTH_SHORT).show();
        }
        try {
            mh = new MagnetometerHandler(manager, magneticRate);
        } catch(InvalidSensorException e) {
            Toast.makeText(context,context.getString(R.string.magnetic_not_available),Toast.LENGTH_SHORT).show();
            mh = null;
        }

        // Initialize WiFi
        WifiManager wifiManager = (WifiManager) context.getSystemService(Context.WIFI_SERVICE);
        if(wifiManager != null)
            wifi = new WifiScanner(wifiManager, wifiRate);
        else {
            Toast.makeText(context,context.getString(R.string.wifi_not_available),Toast.LENGTH_SHORT).show();
            wifi = null;
        }
    }

    @Override
    public void run() {
        final Context context = this.context.get();
        if(context == null)
            return;

        Toast.makeText(
                context,
                context.getString(R.string.compass_calibration_start),
                Toast.LENGTH_SHORT)
                .show();
        initCompass();
        compass.register(new Observer<Heading>() {
            @Override
            public void notify(Heading data) {
                compass.unregister(this);
                Toast.makeText(
                        context,
                        context.getString(R.string.compass_calibration_end),
                        Toast.LENGTH_SHORT)
                        .show();
                start();
            }
        });
    }

    private void initCompass() {
        compass = new RelativeCompass(ah,gh,mh);
    }

    private void start() throws NumberFormatException {
        // Init PDR
        initStepDetection();
        initPDR();
        // Init fingerprints
        loadFingerprints();
        initWifiFingerprint();
        initMagneticFingerprint();
        // Init localization strategy
        initKFStrategy(); // TODO
        // Register observer to strategy
        strategy.unregister(observer);

        // Set flag
        active = true;
    }

    private void initStepDetection() {
        stepDetector = new FasterStepDetector(ah);
    }

    private void initPDR() {
        pdr = new FixedStepPDR(compass, stepDetector, pdrStepLength, pdrInitialHeading);
    }

    private void initWifiFingerprint() {
        int wifiK = 5;
        PositionDistance.Filter filter = null;
        wifiLocalization = new FingerprintStrategy<>(
                floorMap,
                wiFing,
                wifi, wifiK, filter);
    }

    private void initMagneticFingerprint() {
        int magK = 5;
        PositionDistance.Filter filter = null;
        magneticLocalization = new FingerprintStrategy<>(
                floorMap,
                magFing,
                mh, magK, filter);
    }

    private void initKFStrategy() {
        strategy = new KalmanFilterStrategy(
                position,
                floorMap,
                // Inertial
                pdr,
                // Wifi
                wifiDist,
                // Magnetic
                magDist,
                // Wifi filter for MM positions radius
                wifiPositionRadius
        );
    }

/*    private void initPFStrategy() {
        strategy = new ParticleFilterStrategy(
                // Initial position and particles number
                position,
                Constants.PF_PARTICLES_NUMBER,
                floorMap,
                Constants.PDR_STEP_LENGTH,
                pdr,
                wiFing, wifiDist,
                magFing, magDist);
    }
    */

    @Override
    public void stop() {
        if(!active)
            // If finding heading zero, stopEmission (do nothing if compass hasn't been started yet)
            compass.unregisterAll();
        else
            // If active, stopEmission
            stopLocalization();
    }

    private void stopLocalization() {
        // Unregister position update observer
        strategy.unregister(observer);
        // Set flag
        active = false;
    }


    /**
     * Loads fingerprint databases and instantiates the objects.
     */
    private void loadFingerprints() {
        Context context = this.context.get();
        if(context == null)
            return;

        // Wifi fingerprint
        wiFing = (new WifiFingerprintMap.Builder()).build(wiFile);
        wifiDist = new DistancesMap<>(wiFing,wifi,wifiDistancesK,null);

        // Magnetic fingerprint
        magFing = (new MagneticFingerprintMap.Builder()).build(magFile);;
        magDist = new DistancesMap<>(magFing,mh,magneticDistancesK,null);
    }

}
