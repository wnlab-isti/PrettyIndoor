package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.content.Context;
import android.content.SharedPreferences;
import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

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
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.StepDetector;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.MagneticFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.PositionDistance;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.WifiFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.Acceleration;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.AngularSpeed;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.DistancesMap;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.SimpleFingerprintLocalization;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.SimpleKalmanFilterStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.pdr.FixedStepPDR;
import it.cnr.isti.wnlab.indoornavigation.javaonly.pdr.PDR;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.SimpleIndoorParticleFilterStrategy;

public class MainActivity extends AppCompatActivity implements
        CompoundButton.OnCheckedChangeListener,
        View.OnClickListener {

    // TODO dare al service quello che è del service

    /*
     * Localization strategy
     */

    private IndoorLocalizationStrategy strategy;
    private float startX;
    private float startY;
    private FloorMap floorMap;
    private boolean localizing;

    /*
     * GUI members
     */

    // ToggleButtons
    private ToggleButton accToggle;
    private ToggleButton gyroToggle;
    private ToggleButton magToggle;
    private ToggleButton wifiToggle;

    // Radio buttons
    private RadioGroup radioFusionFilters;

    // EditText
    private EditText positionEditText;

    // TextViews
    private TextView wifiPositionTextView;
    private TextView magneticPositionTextView;

    // Floating Action Buttons
    private FloatingActionButton startFab;
    private FloatingActionButton stopFab;
    private FloatingActionButton logStepFab;

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
    private SimpleFingerprintLocalization<AccessPoints> wifiLocalization;
    // Magnetic fingerprint localization. NOT USED EITHER IN PF OR KF
    private SimpleFingerprintLocalization<MagneticField> magneticLocalization;

    /*
     * Logging
     */

    private Observer<IndoorPosition> positionLogger;
    private Observer<IndoorPosition> wifiPositionLogger;
    private Observer<IndoorPosition> magneticPositionLogger;

    private Collection<IndoorPosition> positionsLog;
    private StepLoggerOnDemand stepLogger;

    /*
     * Activity lifecycle
     */

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        localizing = false;
        initializeFilesAndDirectories();
        initializePosition();
        initializeHandlers();
        initializeLogging();
        initializeGUI();
    }

    @Override
    public void onStop() {
        super.onStop();
        if(!localizing)
            // If finding heading zero, stop (do nothing if compass hasn't been started yet)
            compass.unregisterAll();
        else
            // If localizing, stop
            stopLocalization();
    }

    /**
     * Initializes app-related files and directories.
     */
    private void initializeFilesAndDirectories() {
        // Initialize app external folder
        (new File(Constants.APP_FOLDER_PATH_EXTERNAL_STORAGE)).mkdirs();
    }

    /**
     * Initializes start position.
     */
    private void initializePosition() {
        startX = Constants.INITIAL_X;
        startY = Constants.INITIAL_Y;
        floorMap = new PartialISTIFloorMap();
    }

    /**
     * Initializes views and listeners.
     */
    private void initializeGUI() {
        // Radio buttons
        radioFusionFilters = (RadioGroup) findViewById(R.id.radiogroup_fusionfilters);
        radioFusionFilters.check(R.id.radio_kalmanfilter);

        // ToggleButtons
        accToggle = (ToggleButton) findViewById(R.id.toggle_acc);
        gyroToggle = (ToggleButton) findViewById(R.id.toggle_gyro);
        magToggle = (ToggleButton) findViewById(R.id.toggle_mag);
        wifiToggle = (ToggleButton) findViewById(R.id.toggle_wifi);

        // EditText
        positionEditText = (EditText) findViewById(R.id.et_initial_position);
        positionEditText.setText(startX + "," + startY);

        // TextView
        wifiPositionTextView = (TextView) findViewById(R.id.wifi_position);
        magneticPositionTextView = (TextView) findViewById(R.id.mag_position);

        // Floating Action Buttons
        startFab = (FloatingActionButton) findViewById(R.id.fab_start);
        stopFab = (FloatingActionButton) findViewById(R.id.fab_stop);
        logStepFab = (FloatingActionButton) findViewById(R.id.fab_logstep);

        // Set GUI as inactive
        inactiveModeGUI();

        // Eventually set listeners

        accToggle.setOnCheckedChangeListener(this);
        gyroToggle.setOnCheckedChangeListener(this);
        magToggle.setOnCheckedChangeListener(this);
        wifiToggle.setOnCheckedChangeListener(this);

        startFab.setOnClickListener(this);
        stopFab.setOnClickListener(this);
        logStepFab.setOnClickListener(this);
    }

    // ToggleButtons listeners
    @Override
    public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
        if(localizing)
            switch(compoundButton.getId()) {
                // Accelerometer Toggle
                case R.id.toggle_acc:
                    if(ah != null)
                        if(b) {
                            ah.register(accObservers);
                            accObservers = null;
                        } else
                            accObservers = ah.unregisterAll();
                    break;
                // Gyroscope Toggle
                case R.id.toggle_gyro:
                    if(gh != null)
                        if(b) {
                            gh.register(gyroObservers);
                            gyroObservers = null;
                        } else
                            gyroObservers = gh.unregisterAll();
                    break;
                // Magnetometer Toggle
                case R.id.toggle_mag:
                    if(mh != null)
                        if(b) {
                            mh.register(magObservers);
                            magObservers = null;
                        } else
                            magObservers = mh.unregisterAll();
                    break;
                // WifiScanner Toggle
                case R.id.toggle_wifi:
                    if(wifi != null)
                        if(b) {
                            wifi.register(wifiObservers);
                            wifiObservers = null;
                        } else
                            wifiObservers = wifi.unregisterAll();
                    break;
            }
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
    public void onClick(View view) {
        switch(view.getId()) {

            case R.id.fab_start:
                try {
                    // Change GUI
                    activeModeGUI();

                    // Find heading-zero and start localizing
                    setMagneticNorthToEastThenStart();
                } catch(NumberFormatException e) {
                    // EditText text is not well formatted
                    Toast.makeText(this, getString(R.string.invalid_position), Toast.LENGTH_SHORT).show();
                }
                break;

            case R.id.fab_stop:
                stopLocalization();
                break;

            case R.id.fab_logstep:
                stepLogger.log();
                break;
        }
    }

    private void setMagneticNorthToEastThenStart() {
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

        // Initialize stuff and start loggin
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
        wifiLocalization = new SimpleFingerprintLocalization<>(
                floorMap,
                wiFing,
                wifi, wifiK, filter);
    }

    private void initMagneticFingerprint() {
        int magK = 5;
        PositionDistance.Filter filter = null;
        magneticLocalization = new SimpleFingerprintLocalization<>(
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
        strategy = new SimpleKalmanFilterStrategy(
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
        strategy = new SimpleIndoorParticleFilterStrategy(
                // Initial position and particles number
                new XYPosition(startX,startY),
                Constants.PF_PARTICLES_NUMBER,
                floorMap,
                Constants.PDR_STEP_LENGTH,
                pdr,
                wiFing, wifiDist,
                magFing, magDist);
    }

    private void setPositionFromEditText() throws NumberFormatException {
        // Parse EditText content
        String[] split = positionEditText.getText().toString().split(",");
        float newX = Float.parseFloat(split[0]);
        float newY = Float.parseFloat(split[1]);

        // No exceptions: go on
        startX = newX;
        startY = newY;
    }

    private void activeModeGUI() {
        // Set "Stop" and "Log stepDetector" visible
        startFab.setVisibility(View.GONE);
        stopFab.setVisibility(View.VISIBLE);
        logStepFab.setVisibility(View.VISIBLE);

        // Set ToggleButtons as active (where possible)
        if(ah != null){
            accToggle.setActivated(true);
            accToggle.setChecked(true);
        }
        if(gh != null){
            gyroToggle.setActivated(true);
            gyroToggle.setChecked(true);
        }
        if(mh != null){
            magToggle.setActivated(true);
            magToggle.setChecked(true);
        }
        if(wifi != null){
            wifiToggle.setActivated(true);
            wifiToggle.setChecked(true);
        }
    }

    private void inactiveModeGUI() {
        // Set "Start" and "Settings" as visible
        startFab.setVisibility(View.VISIBLE);
        stopFab.setVisibility(View.GONE);
        logStepFab.setVisibility(View.GONE);

        // Set ToggleButtons as inactive
        accToggle.setActivated(false);
        accToggle.setChecked(false);
        gyroToggle.setActivated(false);
        gyroToggle.setChecked(false);
        magToggle.setActivated(false);
        magToggle.setChecked(false);
        wifiToggle.setActivated(false);
        wifiToggle.setChecked(false);
    }

    private void stopLocalization() {
        // Set flag
        localizing = false;
        // Change GUI
        inactiveModeGUI();
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
            int k = (strategy == null || strategy.getClass() == SimpleKalmanFilterStrategy.class ? Constants.KF_WIFI_DISTANCES_K : Constants.PF_WIFI_DISTANCES_K);
            wifiDist = new DistancesMap<>(wiFing,wifi,k,null);
        } else
            Toast.makeText(this,getString(R.string.wifi_fp_db_not_found) + " " + wiFingPath, Toast.LENGTH_LONG).show();

        // Magnetic fingerprint
        String magFingPath = sp.getString(Constants.SP_MAGNETIC_KEY,Constants.SP_MAGNETIC_DEFAULT);
        File magFile = new File(magFingPath);
        if(magFile.exists()) {
            magFing = (new MagneticFingerprintMap.Builder()).build(magFile);
            int k = (strategy == null || strategy.getClass() == SimpleKalmanFilterStrategy.class ? Constants.KF_MAGNETIC_DISTANCES_K : Constants.PF_MAGNETIC_DISTANCES_K);
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