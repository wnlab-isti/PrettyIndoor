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
import android.widget.Toast;
import android.widget.ToggleButton;

import java.io.File;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.android.compass.LawitzkiCompass;
import it.cnr.isti.wnlab.indoornavigation.android.compass.RelativeCompass;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.InvalidSensorException;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.MagnetometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.WifiScanner;
import it.cnr.isti.wnlab.indoornavigation.android.stepdetection.FasterStepDetector;
import it.cnr.isti.wnlab.indoornavigation.android.stepdetection.StepDetector;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.MagneticFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.WifiFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.pdr.FixedLengthPDR;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.pdr.PDR;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization.SimpleIndoorParticleFilterStrategy;

public class MainActivity extends AppCompatActivity implements
        CompoundButton.OnCheckedChangeListener,
        RadioGroup.OnCheckedChangeListener,
        View.OnClickListener {

    // TODO mettere in service quello che deve stare nel service.

    /*
     * Localization strategy
     */

    private IndoorLocalizationStrategy strategy;
    private float startX;
    private float startY;
    private float startFloor;
    private FloorMap floorMap;

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

    // Floating Action Buttons
    private FloatingActionButton startFab;
    private FloatingActionButton stopFab;
    private FloatingActionButton logStepFab;
    private FloatingActionButton settingsFab;

    /*
     * Handlers
     */

    // Sensor handlers
    private AccelerometerHandler ah;
    private GyroscopeHandler gh;
    private MagnetometerHandler mh;

    // Wifi
    private WifiScanner wifi;
    private final int WIFI_RATE = 1400;

    /*
     * PDR
     */
    private LawitzkiCompass compass;
    private StepDetector sd;
    private PDR pdr;

    /*
     * Fingerprints
     */

    private WifiFingerprintMap wiFing;
    private MagneticFingerprintMap magFing;

    /*
     * Logging
     */

    private Observer<IndoorPosition> positionLogger;
    private Collection<IndoorPosition> positionsLog;
    private StepLoggerOnDemand stepLogger;

    /*
     * Activity lifecycle
     */

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initializeFilesAndDirectories();
        initializePosition();
        initializeGUI();
        initializeHandlers();
        loadFingerprints();
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
     * Initializes start position.
     */
    private void initializePosition() {
        startX = Constants.INITIAL_X;
        startY = Constants.INITIAL_Y;
        startFloor = Constants.INITIAL_FLOOR;
        floorMap = new PartialISTIFloorMap();
    }

    /**
     * Initializes views and listeners.
     */
    private void initializeGUI() {
        // ToggleButtons
        accToggle = (ToggleButton) findViewById(R.id.toggle_acc);
        accToggle.setOnCheckedChangeListener(this);
        gyroToggle = (ToggleButton) findViewById(R.id.toggle_gyro);
        gyroToggle.setOnCheckedChangeListener(this);
        magToggle = (ToggleButton) findViewById(R.id.toggle_mag);
        magToggle.setOnCheckedChangeListener(this);
        wifiToggle = (ToggleButton) findViewById(R.id.toggle_wifi);
        wifiToggle.setOnCheckedChangeListener(this);

        // Radio buttons
        radioFusionFilters = (RadioGroup) findViewById(R.id.radiogroup_fusionfilters);
        radioFusionFilters.setOnCheckedChangeListener(this);
        radioFusionFilters.check(R.id.radio_kalmanfilter);

        // EditText
        positionEditText = (EditText) findViewById(R.id.et_initial_position);
        positionEditText.setText(startX + "," + startY);

        // Floating Action Buttons
        startFab = (FloatingActionButton) findViewById(R.id.fab_start);
        startFab.setOnClickListener(this);
        stopFab = (FloatingActionButton) findViewById(R.id.fab_stop);
        stopFab.setOnClickListener(this);
        logStepFab = (FloatingActionButton) findViewById(R.id.fab_logstep);
        logStepFab.setOnClickListener(this);
        settingsFab = (FloatingActionButton) findViewById(R.id.fab_settings);
        settingsFab.setOnClickListener(this);

        // Set GUI as inactive
        inactiveModeGUI();
    }

    // ToggleButtons listeners
    @Override
    public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
        switch(compoundButton.getId()) {
            // Accelerometer Toggle
            case R.id.toggle_acc:
                if(b)
                    ah.start();
                else
                    ah.stop();
                break;
            // Gyroscope Toggle
            case R.id.toggle_gyro:
                if(b)
                    gh.start();
                else
                    gh.stop();
                break;
            // Magnetometer Toggle
            case R.id.toggle_mag:
                if(b)
                    mh.start();
                else
                    mh.stop();
                break;
            // WifiScanner Toggle
            case R.id.toggle_wifi:
                if(b)
                    wifi.start();
                else
                    wifi.stop();
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
            wifi = new WifiScanner(wifiManager, WIFI_RATE);
        else {
            Toast.makeText(this,getString(R.string.wifi_not_available),Toast.LENGTH_SHORT).show();
            wifi = null;
            wifiToggle.setOnClickListener(null);
        }
    }

    // Radio Buttons listeners
    @Override
    public void onCheckedChanged(RadioGroup radioGroup, int i) {
        switch(i) {
            case R.id.radio_kalmanfilter:
                initKalman();
                break;
            case R.id.radio_particlefilter:
                initParticle();
                break;
        }
    }

    private void initKalman() {
// TODO
//        initPDR();
//        strategy = new SimpleIndoorKalmanFilterStrategy(
//                new IndoorPosition(startX,startY,startFloor,System.currentTimeMillis()), pdr);
    }

    private void initParticle() {
        initPDR();
        strategy = new SimpleIndoorParticleFilterStrategy(
                // Initial position and particles number
                new XYPosition(startX,startY),
                Constants.PF_PARTICLES_NUMBER,
                floorMap,
                pdr,
                wifi, wiFing,
                mh, magFing);
    }

    private void initPDR() {
        compass = new RelativeCompass(ah,gh,mh,Constants.PDR_COMPASS_RATE);
        sd = new FasterStepDetector(ah);
        pdr = new FixedLengthPDR(compass,sd,Constants.PDR_INITIAL_HEADING);
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {

            case R.id.fab_start:
                try {
                    // Get first position from EditText
                    setPositionFromEditText();

                    // Change GUI
                    activeModeGUI();

                    // Start handlers
                    startComponents();

                    // Initialize stuff and start loggin
                    startLogging();
                } catch(NumberFormatException e) {
                    // EditText text is not well formatted
                    Toast.makeText(this, getString(R.string.invalid_position), Toast.LENGTH_SHORT).show();
                }
                break;

            case R.id.fab_stop:
                // Change GUI
                inactiveModeGUI();
                // Stop handlers
                stopComponents();
                // Stop logging
                stopLogging();
                break;

            case R.id.fab_settings:
                // TODO
                break;

            case R.id.fab_logstep:
                stepLogger.log();
                break;
        }
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
        // Set "Stop" and "Log step" visible
        startFab.setVisibility(View.GONE);
        stopFab.setVisibility(View.VISIBLE);
        settingsFab.setVisibility(View.GONE);
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
        settingsFab.setVisibility(View.VISIBLE);
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

    private void startComponents() {
        if(ah != null)
            ah.start();
        if(gh != null)
            gh.start();
        if(mh != null)
            mh.start();
        if(wifi != null)
            wifi.start();
        if(compass != null) {
            compass.start();
            Toast.makeText(
                    getApplicationContext(),
                    getString(R.string.compass_calibration_start),
                    Toast.LENGTH_SHORT)
                    .show();
            compass.register(new Observer<Heading>() {
                @Override
                public void notify(Heading data) {
                    compass.unregister(this);
                    Toast.makeText(
                            getApplicationContext(),
                            getString(R.string.compass_calibration_end),
                            Toast.LENGTH_SHORT)
                            .show();
                }
            });
        }
    }

    private void stopComponents() {
        if(ah != null)
            ah.stop();
        if(gh != null)
            gh.stop();
        if(mh != null)
            mh.stop();
        if(wifi != null)
            wifi.stop();
        if(compass != null)
            compass.stop();
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
        if(wiFile.exists())
            wiFing = (new WifiFingerprintMap.Builder()).build(wiFile);
        else
            Toast.makeText(this,getString(R.string.wifi_fp_db_not_found) + " " + wiFingPath, Toast.LENGTH_LONG).show();

        // Magnetic fingerprint
        String magFingPath = sp.getString(Constants.SP_MAGNETIC_KEY,Constants.SP_MAGNETIC_DEFAULT);
        File magFile = new File(magFingPath);
        if(magFile.exists())
            magFing = (new MagneticFingerprintMap.Builder()).build(magFile);
        else
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
    }

    private void startLogging() {
        // Register logger for dialog
        strategy.register(positionLogger);

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

        // Build dialog content
        DecimalFormat df = new DecimalFormat("#.#");
        StringBuilder log = new StringBuilder();
        for(IndoorPosition p : positionsLog)
            log.append(df.format(p.x)).append(",").append(df.format(p.y)).append("\n");

        // Show alert
        new AlertDialog.Builder(this)
                .setTitle("Positions at each step")
                .setMessage(log.toString())
                .setIcon(android.R.drawable.ic_dialog_info)
                .show();

        // Close step logger writer
        try {
            stepLogger.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}