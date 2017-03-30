package it.cnr.isti.wnlab.indoornavigation.utils.localization.kalmanfilter;

import android.content.Context;
import android.content.SharedPreferences;
import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.support.v7.app.AlertDialog;
import android.widget.Toast;

import java.io.File;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.android.compass.LawitzkiCompass;
import it.cnr.isti.wnlab.indoornavigation.android.compass.RelativeCompass;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.InvalidSensorException;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.MagnetometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.WifiScanner;
import it.cnr.isti.wnlab.indoornavigation.android.stepdetection.FasterStepDetector;
import it.cnr.isti.wnlab.indoornavigation.androidapp.localization.Constants;
import it.cnr.isti.wnlab.indoornavigation.androidapp.localization.StepLoggerOnDemand;
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
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.Acceleration;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.AngularSpeed;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.fingerprint.FingerprintStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.particlefilter.IndoorParticleFilterStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.pdr.FixedStepPDR;

public class NavigatorWithKalmanFilter implements IndoorNavigator {

    /*
     * Localization strategy
     */

    private IndoorLocalizationStrategy strategy;
    private float startX;
    private float startY;
    private FloorMap floorMap;
    private boolean localizing;

    /*
     * Handlers
     */

    // Sensor handlers
    private AccelerometerHandler ah;
    private List<Observer<Acceleration>> accObservers;
    private GyroscopeHandler gh;
    private List<Observer<AngularSpeed>> gyroObservers;
    private MagnetometerHandler mh;
    private List<Observer<MagneticField>> magObservers;

    // Wifi
    private WifiScanner wifi;
    private List<Observer<AccessPoints>> wifiObservers;

    /*
     * PDR
     */

    private LawitzkiCompass compass;
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

    /*
     * Logging
     */

    private Observer<IndoorPosition> positionLogger;
    private Observer<IndoorPosition> wifiPositionLogger;
    private Observer<IndoorPosition> magneticPositionLogger;

    private Collection<IndoorPosition> positionsLog;
    private StepLoggerOnDemand stepLogger;

    public NavigatorWithKalmanFilter() {
        localizing = false;
        initializeFilesAndDirectories();
        initializePosition();
        initializeHandlers();
        initializeLogging();
    }


    /**
     * Initializes app-related files and directories.
     */
    private void initializeFilesAndDirectories() {
        // Initialize app external folder
        (new File(Constants.APP_FOLDER_PATH_EXTERNAL_STORAGE)).mkdirs();
    }

    /**
     * Initializes startEmission position.
     */
    private void initializePosition() {
        startX = Constants.INITIAL_X;
        startY = Constants.INITIAL_Y;
        floorMap = new PartialISTIFloorMap();
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
            Toast.makeText(this,getString(R.string.acc_not_available),Toast.LENGTH_SHORT).show();
            ah = null;
            accToggle.setOnClickListener(null);
        }
        try {
            gh = new GyroscopeHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        } catch(InvalidSensorException e) {
            Toast.makeText(this,getString(R.string.gyro_not_available),Toast.LENGTH_SHORT).show();
            gh = null;
            gyroToggle.setOnClickListener(null);
        }
        try {
            mh = new MagnetometerHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        } catch(InvalidSensorException e) {
            Toast.makeText(this,getString(R.string.magnetic_not_available),Toast.LENGTH_SHORT).show();
            mh = null;
            gyroToggle.setOnClickListener(null);
        }

        // Initialize WiFi
        WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        if(wifiManager != null)
            wifi = new WifiScanner(wifiManager, Constants.WIFI_RATE);
        else {
            Toast.makeText(this,getString(R.string.wifi_not_available),Toast.LENGTH_SHORT).show();
            wifi = null;
            wifiToggle.setOnClickListener(null);
        }
    }

    @Override
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
        // Get first position from EditText (throws NumberFormatException if text isn't valid)
        setPositionFromEditText();

        // Init PDR
        initStepDetection();
        initPDR();
        // Init fingerprints
        loadFingerprints();
        initWifiFingerprint();
        initMagneticFingerprint();
        // Init localization strategy
        initStrategy();

        // Initialize stuff and startEmission loggin
        startLogging();

        // Set flag
        localizing = true;
    }


    private void initStepDetection() {
        stepDetector = new FasterStepDetector(ah);
    }

    private void initPDR() {
        pdr = new FixedStepPDR(compass, stepDetector, Constants.PDR_STEP_LENGTH, Constants.PDR_INITIAL_HEADING);
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

    private void initStrategy() {
        switch(radioFusionFilters.getCheckedRadioButtonId()) {
            case R.id.radio_kalmanfilter:
                initKFStrategy();
                break;
            case R.id.radio_particlefilter:
                initPFStrategy();
                break;
            default:
                Toast.makeText(this, "Which strategy do you want to use?", Toast.LENGTH_SHORT).show();
        }
    }

    private void initKFStrategy() {
        strategy = new KalmanFilterStrategy(
                new XYPosition(startX,startY),
                floorMap,
                // Inertial
                pdr,
                // Wifi
                wifiDist,
                // Magnetic
                magDist,
                // Wifi filter for MM positions radius
                Constants.KF_WIFI_POSITION_RADIUS
        );
    }

    private void initPFStrategy() {
        strategy = new IndoorParticleFilterStrategy(
                // Initial position and particles number
                new XYPosition(startX,startY),
                Constants.PF_PARTICLES_NUMBER,
                floorMap,
                Constants.PDR_STEP_LENGTH,
                pdr,
                wiFing, wifiDist,
                magFing, magDist);
    }

    @Override
    public void stop() {
        if(!localizing)
            // If finding heading zero, stopEmission (do nothing if compass hasn't been started yet)
            compass.unregisterAll();
        else
            // If localizing, stopEmission
            stopLocalization();
    }

    private void stopLocalization() {
        // Set flag
        localizing = false;
        // Stop logging
        stopLogging();
    }


    /**
     * Loads fingerprint databases and instantiates the objects.
     */
    private void loadFingerprints() {
        // Shared Preferences for retrieving paths
        SharedPreferences sp = getSharedPreferences(Constants.SP_NAME, MODE_PRIVATE);

        // Wifi fingerprint
        String wiFingPath = sp.getString(Constants.SP_WIFIFP_KEY,Constants.SP_WIFIFP_DEFAULT);
        File wiFile = new File(wiFingPath);
        if(wiFile.exists()) {
            wiFing = (new WifiFingerprintMap.Builder()).build(wiFile);
            int k = (strategy == null || strategy.getClass() == KalmanFilterStrategy.class ? Constants.KF_WIFI_DISTANCES_K : Constants.PF_WIFI_DISTANCES_K);
            wifiDist = new DistancesMap<>(wiFing,wifi,k,null);
        } else
            Toast.makeText(this,getString(R.string.wifi_fp_db_not_found) + " " + wiFingPath, Toast.LENGTH_LONG).show();

        // Magnetic fingerprint
        String magFingPath = sp.getString(Constants.SP_MAGNETIC_KEY,Constants.SP_MAGNETIC_DEFAULT);
        File magFile = new File(magFingPath);
        if(magFile.exists()) {
            magFing = (new MagneticFingerprintMap.Builder()).build(magFile);
            int k = (strategy == null || strategy.getClass() == KalmanFilterStrategy.class ? Constants.KF_MAGNETIC_DISTANCES_K : Constants.PF_MAGNETIC_DISTANCES_K);
            magDist = new DistancesMap<>(magFing,mh,k,null);
        } else
            Toast.makeText(this, getString(R.string.mag_fp_db_not_found)  + " " + magFingPath, Toast.LENGTH_LONG).show();
    }

    /**
     * Initialize objects for logging.
     */
    private void initializeLogging() {
        // Alert logger (all positions)
        positionsLog = new ArrayList<>();
        positionLogger = new Observer<IndoorPosition>() {
            @Override
            public void notify(IndoorPosition data) {
                positionsLog.add(data);
                positionEditText.setText(data.x + "," + data.y);
            }
        };

        // Wifi-only position observer
        wifiPositionLogger = new Observer<IndoorPosition>(){

            @Override
            public void notify(IndoorPosition data) {
                wifiPositionTextView.setText(data.x + "," + data.y);
            }

        };

        // MM-only position observer
        magneticPositionLogger = new Observer<IndoorPosition>() {

            @Override
            public void notify(IndoorPosition data) {
                magneticPositionTextView.setText(data.x + "," + data.y);
            }

        };
    }

    private void startLogging() {
        // Register logger for dialog
        strategy.register(positionLogger);

        // Register loggers for TextViews update
        wifiLocalization.register(wifiPositionLogger);
        magneticLocalization.register(magneticPositionLogger);

        // Step logger (FAB)
        SharedPreferences sp = getSharedPreferences(Constants.SP_NAME, MODE_PRIVATE);
        String logFolderPath =
                sp.getString(Constants.SP_LOG_FOLDER_KEY, Constants.SP_LOG_DEFAULT);
        (new File(logFolderPath)).mkdirs();
        try {
            stepLogger = new StepLoggerOnDemand(strategy, logFolderPath);
        } catch(IOException e) {
            Toast.makeText(this, "Can't write on log file: " + e.getLocalizedMessage(), Toast.LENGTH_SHORT).show();
            e.printStackTrace();
        }
    }

    private void stopLogging() {
        // Unregister logger
        strategy.unregister(positionLogger);

        // Unregister fingerprint TextView-updating loggers
        wifiLocalization.unregister(wifiPositionLogger);
        magneticLocalization.unregister(magneticPositionLogger);

        // Build dialog content
        DecimalFormat df = new DecimalFormat("#.#");
        StringBuilder log = new StringBuilder();
        for(IndoorPosition p : positionsLog)
            log.append(df.format(p.x)).append(",").append(df.format(p.y)).append("\n");

        // Show alert
        new AlertDialog.Builder(this)
                .setTitle("Positions at each stepDetector")
                .setMessage(log.toString())
                .setIcon(android.R.drawable.ic_dialog_info)
                .show();

        // Close stepDetector logger writer
        try {
            stepLogger.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
