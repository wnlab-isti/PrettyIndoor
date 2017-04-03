package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.os.Binder;
import android.os.IBinder;
import android.widget.Toast;

import java.io.File;

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
import it.cnr.isti.wnlab.indoornavigation.utils.localization.particlefilter.ParticleFilterStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.pdr.PDRStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.pdr.FixedStepPDR;

public class SimpleIndoorService extends Service implements Observer<IndoorPosition> {

    /*********************************************************
     * Service
     *********************************************************/

    public static boolean active = false;

    public static final String INTENT_STRATEGY_CHOICE = "strategy";
    public static final String INTENT_START_POSITION = "position";
    public static final String INTENT_LOGGING = "log";

    private IndoorPosition lastPosition;

    // The object that client receives
    private final IBinder mBinder = new SimpleBinder();

    /**
     * Class for clients to access.  Because we know this service always
     * runs in the same process as its clients, we don't need to deal with
     * IPC.
     */
    public class SimpleBinder extends Binder {
        SimpleBinder getService() {
            return SimpleBinder.this;
        }
        IndoorPosition getPosition() {
            return lastPosition;
        }
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        // Set initial paramaters
        chosenStrategy = (Strategies) intent.getExtras().get(INTENT_STRATEGY_CHOICE);
        position = (XYPosition) intent.getExtras().get(INTENT_START_POSITION);
        floorMap = new PartialISTIFloorMap();
        lastPosition = new IndoorPosition(position, floorMap.getFloor(), System.currentTimeMillis());
        // Initialize and start localization
        initialize();
        run();
        // Observe position updates (and actually make the whole thing start running)
        strategy.register(this);

        // Set service as active (for clients)
        active = true;

        return START_STICKY;
    }

    @Override
    public void notify(IndoorPosition data) {
        this.lastPosition = data;
    }

    @Override
    public void onDestroy() {
        // Stop localization (if any)
        if(localizationIsActive) {
            strategy.unregister(this);
            stop();
        }

        // Set service as inactive (for clients)
        active = false;

        // Tell the user we stopped.
        Toast.makeText(this, R.string.service_stopped, Toast.LENGTH_SHORT).show();
    }


    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

    /*********************************************************
     * Logic
     *********************************************************/

    /*
     * Localization strategy
     */

    private IndoorLocalizationStrategy strategy;
    private XYPosition position;
    private FloorMap floorMap;
    private boolean localizationIsActive;

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
    private float pdrStepLength;
    private float pdrInitialHeading;

    /*
     * Fingerprints
     */

    // Wifi

    private File wiFile;
    private float wifiPositionRadius;
    private long wifiRate;
    private int kfWifiDistancesK;
    private int pfWifiDistancesK;
    private WifiFingerprintMap wiFing;
    private DistancesMap<XYPosition, AccessPoints> wifiDist;

    // Magnetic

    private File magFile;
    private int magneticRate;
    private int kfMagneticDistancesK;
    private int pfMagneticDistancesK;
    private MagneticFingerprintMap magFing;
    private DistancesMap<XYPosition, MagneticField> magDist;

    /*
     * Constants and strategy selection
     */

    public enum Strategies {
        PDR_STRATEGY,
        WIFIFP_STRATEGY,
        MAGFP_STRATEGY,
        PF_STRATEGY,
        KF_STRATEGY
    }

    private Strategies chosenStrategy;

    /**********************************************
     * Initialization
     **********************************************/

    private void initialize() {
        initializeParameters();
        initializeFilesAndDirectories();
        initializeHandlers();
    }

    private void initializeParameters() {
        // PDR parameters
        this.pdrStepLength = Constants.PDR_STEP_LENGTH;
        this.pdrInitialHeading = Constants.PDR_INITIAL_HEADING;
        // Wifi parameters
        this.wifiRate = Constants.WIFI_RATE;
        this.wifiPositionRadius = Constants.KF_WIFI_POSITION_RADIUS;
        // Magnetic parameters
        this.magneticRate = SensorManager.SENSOR_DELAY_FASTEST;
        // Parameters depending on the chosen strategy
        switch(chosenStrategy) {
            case KF_STRATEGY:
                this.kfWifiDistancesK = Constants.KF_WIFI_DISTANCES_K;
                this.kfMagneticDistancesK = Constants.KF_MAGNETIC_DISTANCES_K;
                break;
            case PF_STRATEGY:
            case PDR_STRATEGY:
            case WIFIFP_STRATEGY:
            case MAGFP_STRATEGY:
                this.pfWifiDistancesK = Constants.PF_WIFI_DISTANCES_K;
                this.pfMagneticDistancesK = Constants.PF_MAGNETIC_DISTANCES_K;
        }
        // Set localization flag as inactive
        this.localizationIsActive = false;
    }

    private void initializeFilesAndDirectories() {
        this.wiFile = new File(Constants.WIFIFP_DEFAULT);
        this.magFile = new File(Constants.MAGFP_DEFAULT);
    }

    /**
     * Initializes sensor and wifi handlers. If they are not available, the related ToggleButton is
     * deactivated.
     */
    private void initializeHandlers() {
        // Initialize sensors and handlers
        SensorManager manager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        // Accelerometer
        try {
            ah = new AccelerometerHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        } catch(InvalidSensorException e) {
            Toast.makeText(getApplicationContext(), getString(R.string.acc_not_available),Toast.LENGTH_SHORT).show();
            ah = null;
        }
        try {
            gh = new GyroscopeHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        } catch(InvalidSensorException e) {
            Toast.makeText(getApplicationContext(), getString(R.string.gyro_not_available),Toast.LENGTH_SHORT).show();
            gh = null;
        }
        try {
            mh = new MagnetometerHandler(manager, magneticRate);
        } catch(InvalidSensorException e) {
            Toast.makeText(getApplicationContext(), getString(R.string.magnetic_not_available),Toast.LENGTH_SHORT).show();
            mh = null;
        }

        // Initialize WiFi
        WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        if(wifiManager != null)
            wifi = new WifiScanner(wifiManager, wifiRate);
        else {
            Toast.makeText(getApplicationContext(), getString(R.string.wifi_not_available),Toast.LENGTH_SHORT).show();
            wifi = null;
        }
    }

    /**********************************************
     * Run
     **********************************************/

    public void run() {
        Toast.makeText(
                getApplicationContext(),
                getString(R.string.compass_calibration_start),
                Toast.LENGTH_SHORT)
                .show();
        initCompass();
        compass.register(new Observer<Heading>() {
            @Override
            public void notify(Heading data) {
                compass.unregister(this);
                Toast.makeText(
                        getApplicationContext(),
                        getString(R.string.compass_calibration_end),
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
        // NOTE: compass is already initalized here

        // Init localization strategy
        switch(chosenStrategy) {
            case PDR_STRATEGY:
                // Localization with PDR only
                initPDRStrategy();
                break;
            case WIFIFP_STRATEGY:
                // Localization with Wifi fingerprinting
                initWifiFingerprintStrategy();
                break;
            case MAGFP_STRATEGY:
                // Localization with Magnetic fingerprinting
                initMagneticFingerprintStrategy();
                break;
            case KF_STRATEGY:
                // Localization with KF: it depends on PDR and fingerprints
                initKFStrategy();
                break;
            case PF_STRATEGY:
                // Localization with PF: it depends on PDR and fingerprints
                initPFStrategy();
                break;
        }

        // Set flag
        localizationIsActive = true;
    }

    private void initPDRStrategy() {
        // Initialize middle components
        initStepDetection();
        initPDR();
        // Initialize strategy instance
        strategy = new PDRStrategy(
                pdr,
                position,
                floorMap
        );
    }

    private void initWifiFingerprintStrategy() {
        // Initialize middle components
        initFingerprints();
        initWifiFingerprint();
        int k = 5;
        PositionDistance.Filter knnFilter = null;
        // Initialize strategy instance
        strategy = new FingerprintStrategy<>(
                floorMap,
                wiFing,
                wifi,
                k, knnFilter);
    }

    private void initMagneticFingerprintStrategy() {
        // Initialize middle components
        initFingerprints();
        int k = 5;
        PositionDistance.Filter knnFilter = null;
        // Initialize strategy instance
        strategy = new FingerprintStrategy<>(
                floorMap,
                magFing,
                mh,
                k, knnFilter);
    }

    private void initKFStrategy() {
        // Initialize middle components
        initStepDetection();
        initPDR();
        initFingerprints();
        // Initialize strategy instance
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

    private void initPFStrategy() {
        // Initialize middle components
        initStepDetection();
        initPDR();
        initFingerprints();
        initWifiFingerprint();
        initMagneticFingerprint();
        // Initialize strategy instance
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

    private void initStepDetection() {
        stepDetector = new FasterStepDetector(ah);
    }

    private void initPDR() {
        pdr = new FixedStepPDR(compass, stepDetector, pdrStepLength, pdrInitialHeading);
    }

    /**
     * Loads fingerprint databases and instantiates the objects.
     */
    private void initFingerprints() {
        // Wifi fingerprint
        wiFing = (new WifiFingerprintMap.Builder()).build(wiFile);
        wifiDist = new DistancesMap<>(wiFing, wifi, (chosenStrategy == Strategies.KF_STRATEGY ? kfWifiDistancesK : pfWifiDistancesK), null);

        // Magnetic fingerprint
        magFing = (new MagneticFingerprintMap.Builder()).build(magFile);
        magDist = new DistancesMap<>(magFing, mh, (chosenStrategy == Strategies.KF_STRATEGY ? kfMagneticDistancesK : pfMagneticDistancesK), null);
    }

    private void initWifiFingerprint() {
        int k = 5;
        PositionDistance.Filter knnFilter = null;
        strategy = new FingerprintStrategy<>(
                floorMap,
                wiFing,
                wifi, k, knnFilter);
    }

    private void initMagneticFingerprint() {
        int magK = 5;
        strategy = new FingerprintStrategy<>(
                floorMap,
                magFing,
                mh, magK, null);
    }

    /**********************************************
     * Stop
     **********************************************/

    public void stop() {
        if(!localizationIsActive)
            // If finding heading zero, stopEmission (do nothing if compass hasn't been started yet)
            compass.unregisterAll();
        else
            // If localizationIsActive, stopEmission
            stopLocalization();
    }

    private void stopLocalization() {
        // Set flag
        localizationIsActive = false;
    }

}
