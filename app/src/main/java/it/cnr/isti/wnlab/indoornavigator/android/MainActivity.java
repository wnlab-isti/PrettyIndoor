package it.cnr.isti.wnlab.indoornavigator.android;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.RadioGroup;
import android.widget.Switch;
import android.widget.TextView;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.ArrayList;

import it.cnr.isti.wnlab.indoornavigator.R;
import it.cnr.isti.wnlab.indoornavigator.androidutils.IndoorNavigator;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.MagneticFieldHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.wifi.WifiScanner;
import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.StartableStoppable;

public class MainActivity extends AppCompatActivity implements
        CompoundButton.OnCheckedChangeListener,
        View.OnClickListener {

    // IndoorNavigator instance
    private IndoorNavigator mNav;
    private float INITIAL_X = 19.5f;
    private float INITIAL_Y = 5.7f;
    private int INITIAL_FLOOR = 0;
    private IndoorPosition mInitialPosition =
            new IndoorPosition(INITIAL_X,INITIAL_Y,INITIAL_FLOOR,System.currentTimeMillis());

    // Switches
    private Switch accSwitch;
    private Switch gyroSwitch;
    private Switch magSwitch;
    private Switch wifiSwitch;
    private Switch logSwitch;

    // Radio buttons
    private RadioGroup radios;

    // Buttons
    private Button btnStart;
    private Button btnWifi;
    private Button btnMM;
    private Button btnLogPosition;

    // Sensor handlers
    private AccelerometerHandler ah;
    private GyroscopeHandler gh;
    private MagneticFieldHandler mh;

    // Wifi
    private WifiScanner wifi;
    private final int WIFI_RATE = 1400;

    // Fingerprint maps
    private File wifiFingerprintDB;
    private File magneticFingerprintDB;

    // Log and output
    private StepLoggerObserver mStepLoggerObserver;
    private Writer mLogWriter;

    // Shared Preferences constants
    private String SHARED_PREFERENCES_FILE = "PIN_FILEPATHS";
    private String SHARED_PREFERENCES_WIFI_DB_PATH = "WIFI_DB_PATH";
    private String SHARED_PREFERENCES_MM_DB_PATH = "MM_DB_PATH";

    // TextView
    private TextView textWifiDB;
    private TextView textMMDB;

    // Activity request codes
    private static final int WIFI_REQUEST_CODE = 42;
    private static final int MAGNETIC_REQUEST_CODE = 1337;

    // Writer for logging
    private final String RAW_LOG_FILE_NAME = "pin_raw" + System.currentTimeMillis() + ".log";

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Initialize app external folder
        (new File(Environment.getExternalStorageDirectory() + "/" + getString(R.string.app_name)))
                .mkdirs();

        // Initialize sensors and handlers
        SensorManager manager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        ah = new AccelerometerHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        gh = new GyroscopeHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        mh = new MagneticFieldHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);

        // Initialize WiFi
        WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        wifi = new WifiScanner(wifiManager, WIFI_RATE);

        // Initialize GUI pointers

        // Switches
        accSwitch = (Switch) findViewById(R.id.switch_acc);
        accSwitch.setOnCheckedChangeListener(this);
        gyroSwitch = (Switch) findViewById(R.id.switch_gyro);
        gyroSwitch.setOnCheckedChangeListener(this);
        magSwitch = (Switch) findViewById(R.id.switch_mag);
        magSwitch.setOnCheckedChangeListener(this);
        wifiSwitch = (Switch) findViewById(R.id.switch_wifi);
        wifiSwitch.setOnCheckedChangeListener(this);
        logSwitch = (Switch) findViewById(R.id.switch_log);

        // Radio buttons
        radios = (RadioGroup) findViewById(R.id.radiog_mode);

        // Buttons
        btnStart = (Button) findViewById(R.id.btn_start);
        btnStart.setOnClickListener(this);
        btnWifi = (Button) findViewById(R.id.btn_load_wifi_db);
        btnWifi.setOnClickListener(this);
        btnMM = (Button) findViewById(R.id.btn_load_mm_db);
        btnMM.setOnClickListener(this);
        btnLogPosition = (Button) findViewById(R.id.btn_log_position);
        btnLogPosition.setOnClickListener(this);

        // TextViews
        textWifiDB = (TextView) findViewById(R.id.wifidb_text);
        textMMDB = (TextView) findViewById(R.id.mmdb_text);
        initFingerprintLoad();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (resultCode == RESULT_OK) {
            switch(requestCode) {
                // From WifiDB file request
                case WIFI_REQUEST_CODE:
                    setWifiDB(data.getData().getPath());
                    break;
                // From geomagnetic DB file request
                case MAGNETIC_REQUEST_CODE:
                    setMMDB(data.getData().getPath());
                    break;
            }
        }
    }

    private void setWifiDB(String path) {
        wifiFingerprintDB = new File(path);
        getSharedPreferences(SHARED_PREFERENCES_FILE, Context.MODE_PRIVATE).edit()
                .putString(SHARED_PREFERENCES_WIFI_DB_PATH,path).commit();
        textWifiDB.setText(path);
    }

    private void setMMDB(String path) {
        magneticFingerprintDB = new File(path);
        getSharedPreferences(SHARED_PREFERENCES_FILE, Context.MODE_PRIVATE).edit()
                .putString(SHARED_PREFERENCES_MM_DB_PATH,path).commit();
        textMMDB.setText(path);
    }

    private void chooseFileRequest(int requestCode) {
        Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
        intent.setType("text/plain");
        startActivityForResult(intent, requestCode);
    }

    private void initFingerprintLoad() {
        SharedPreferences pref = getSharedPreferences(SHARED_PREFERENCES_FILE, Context.MODE_PRIVATE);
        // Wifi
        String wifiPath = pref.getString(SHARED_PREFERENCES_WIFI_DB_PATH, null);
        if(wifiPath != null) {
            wifiFingerprintDB = new File(wifiPath);
            textWifiDB.setText(wifiPath);
        }
        // MM
        String mmPath = pref.getString(SHARED_PREFERENCES_MM_DB_PATH, null);
        if(mmPath != null) {
            magneticFingerprintDB = new File(mmPath);
            textMMDB.setText(mmPath);
        }
    }

    @Override
    public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
        switch(compoundButton.getId()) {
            // Accelerometer switch
            case R.id.switch_acc:
                if(ah != null)
                    if(b)
                        ah.start();
                    else
                        ah.stop();
                break;

            // Gyroscope switch
            case R.id.switch_gyro:
                if(gh != null)
                    if(b)
                        gh.start();
                    else
                        gh.stop();
                break;

            // Magnetometer switch
            case R.id.switch_mag:
                if(mh != null)
                    if(b)
                        mh.start();
                    else
                        mh.stop();
                break;

            // Wifi scanner switch
            case R.id.switch_wifi:
                if(wifi != null)
                    if(b)
                        wifi.start();
                    else
                        wifi.stop();
                break;
        }
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.btn_start:
                if(mNav == null || !mNav.isStarted()) {
                    mNav = initNavigator(radios.getCheckedRadioButtonId());
                    mNav.start();
                    ((Button) view).setText("STOP");
                } else {
                    mNav.stop();
                    mNav = null;
                    resetSwitches();
                    closeWriters();
                    ((Button) view).setText("START");
                }
                break;

            case R.id.btn_load_wifi_db:
                chooseFileRequest(WIFI_REQUEST_CODE);
                break;

            case R.id.btn_load_mm_db:
                chooseFileRequest(MAGNETIC_REQUEST_CODE);
                break;

            case R.id.btn_log_position:
                mStepLoggerObserver.steplog();
                break;
        }
    }

    private IndoorNavigator initNavigator(int radio) {
        // Builder for Indoor Navigator
        IndoorNavigator.Builder builder = new IndoorNavigator.Builder(this);

        // Set initial position
        builder.setInitialPosition(mInitialPosition);

        // Add data sources
        ArrayList<StartableStoppable> sources = new ArrayList<>();
        // KF (PDR/WIFI/MM)
        if(radio == R.id.radio_kf) {
            sources.add(ah);
            if(ah != null)
                accSwitch.setChecked(true);
            sources.add(gh);
            if(gh != null)
                gyroSwitch.setChecked(true);
        }
        // KF or Wifi
        if(radio == R.id.radio_kf || radio == R.id.radio_wifi_only) {
            if(wifi != null) {
                // Add Wifi source
                sources.add(wifi);
                // Set fingerprint file
                builder.setWifiFingerprintMap(wifiFingerprintDB);
                // Change status of the wifi switch
                wifiSwitch.setChecked(true);
            }
        }
        // KF or MM
        if(radio == R.id.radio_kf || radio == R.id.radio_mm_only) {
            if(mh != null) {
                // Add Magnetic source
                sources.add(mh);
                // Set fingerprint file
                builder.setMagneticFingerprintMap(magneticFingerprintDB);
                // Change status of the magnetometer switch
                magSwitch.setChecked(true);
            }
        }

        // Set sources
        builder.setSources(sources);

        // Initialize output observer
        mStepLoggerObserver = new StepLoggerObserver(mInitialPosition);
        builder.setPositionObserver(mStepLoggerObserver);

        // If checked, initialize data log
        if(logSwitch.isChecked()) {
            File rawLogFile = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + getString(R.string.app_name) + "/" + RAW_LOG_FILE_NAME);
            try {
                mLogWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(rawLogFile)));
                builder.setLogWriter(mLogWriter);
                builder.setWifiFingerprintLogger(new PositionLogger(mLogWriter));
                builder.setMMFingerprintLogger(new PositionLogger(mLogWriter));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        // Make navigator
        return builder.create();
    }

    private void resetSwitches() {
        accSwitch.setChecked(false);
        gyroSwitch.setChecked(false);
        magSwitch.setChecked(false);
        wifiSwitch.setChecked(false);
        logSwitch.setChecked(false);
    }

    @Override
    public void onStop() {
        super.onStop();
        closeWriters();
    }

    private void closeWriters() {
        if(mLogWriter != null)
            try {
                mLogWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        if(mStepLoggerObserver != null)
            try {
                mStepLoggerObserver.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
    }
}
