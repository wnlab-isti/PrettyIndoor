package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.content.Context;
import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.RadioGroup;
import android.widget.ToggleButton;

import java.io.File;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.MagneticFieldHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.WifiScanner;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.MagneticFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.WifiFingerprintMap;

public class MainActivity extends AppCompatActivity implements
        CompoundButton.OnCheckedChangeListener,
        RadioGroup.OnCheckedChangeListener,
        View.OnClickListener {

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
    private MagneticFieldHandler mh;

    // Wifi
    private WifiScanner wifi;
    private final int WIFI_RATE = 1400;

    /*
     * Fingerprints
     */

    private WifiFingerprintMap wiFing;
    private MagneticFingerprintMap magFing;

    /*
     * Activity lifecycle
     */

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initializeFilesAndDirectories();
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

        // Floating Action Buttons
        startFab = (FloatingActionButton) findViewById(R.id.fab_start);
        startFab.setOnClickListener(this);
        stopFab = (FloatingActionButton) findViewById(R.id.fab_stop);
        stopFab.setOnClickListener(this);
        logStepFab = (FloatingActionButton) findViewById(R.id.fab_logstep);
        logStepFab.setOnClickListener(this);
        settingsFab = (FloatingActionButton) findViewById(R.id.fab_settings);
        settingsFab.setOnClickListener(this);
    }

    @Override
    public void onCheckedChanged(CompoundButton compoundButton, boolean b) {

    }

    @Override
    public void onCheckedChanged(RadioGroup radioGroup, int i) {

    }

    @Override
    public void onClick(View view) {

    }

    private void initializeHandlers() {
        // Initialize sensors and handlers
        SensorManager manager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        ah = new AccelerometerHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        gh = new GyroscopeHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        mh = new MagneticFieldHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);

        // Initialize WiFi
        WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        wifi = new WifiScanner(wifiManager, WIFI_RATE);
    }

    private void loadFingerprints() {
        // TODO
    }

    private void initializeLogging() {
        // TODO
    }

}
