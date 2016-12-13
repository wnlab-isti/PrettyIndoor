package it.cnr.isti.wnlab.indoornavigator.android;

import android.content.Context;
import android.content.Intent;
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

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

import it.cnr.isti.wnlab.indoornavigator.R;
import it.cnr.isti.wnlab.indoornavigator.androidutils.IndoorNavigator;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.MagneticFieldHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.wifi.WifiScanner;
import it.cnr.isti.wnlab.indoornavigator.framework.StartableStoppable;

public class MainActivity extends AppCompatActivity implements
        CompoundButton.OnCheckedChangeListener,
        View.OnClickListener {

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

    // Sensor handlers
    private AccelerometerHandler ah;
    private GyroscopeHandler gh;
    private MagneticFieldHandler mh;

    // Wifi
    private WifiScanner wifi;
    private final int WIFI_RATE = 30;

    // Fingerprint maps
    private File wifiFingerprintDB;
    private File magneticFingerprintDB;

    // Activity request codes
    private static final int WIFI_REQUEST_CODE = 42;
    private static final int MAGNETIC_REQUEST_CODE = 1337;

    // Writer for logging
    private final String RAW_LOG_FILE_NAME = "pin_raw.log";

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Initialize sensors and handlers
        SensorManager manager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        ah = new AccelerometerHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        gh = new GyroscopeHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);
        mh = new MagneticFieldHandler(manager, SensorManager.SENSOR_DELAY_FASTEST);

        // Initialize WiFi
        WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        wifi = new WifiScanner(wifiManager, WIFI_RATE);

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
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (resultCode == RESULT_OK) {
            switch(requestCode) {
                // From WifiDB file request
                case WIFI_REQUEST_CODE:
                    wifiFingerprintDB = new File(data.getData().getPath());
                    break;
                // From geomagnetic DB file request
                case MAGNETIC_REQUEST_CODE:
                    magneticFingerprintDB = new File(data.getData().getPath());
                    break;
            }
        }
    }

    private void chooseFileRequest(int requestCode) {
        Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
        intent.setType("text/plain");
        startActivityForResult(intent, requestCode);
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
                initNavigator();
                break;
        }
    }

    private IndoorNavigator initNavigator() {
        // Builder for Indoor Navigator
        IndoorNavigator.Builder builder = new IndoorNavigator.Builder();

        // Add data sources
        ArrayList<StartableStoppable> sources = new ArrayList<>();
        sources.add(ah);
        sources.add(gh);
        sources.add(mh);
        sources.add(wifi);
        builder.setSources(sources);

        // Set fingerprint files
        builder.setWifiFingerprintMap(wifiFingerprintDB);
        builder.setMagneticFingerprintMap(magneticFingerprintDB);

        // Set updater
        builder.setPositionObserver(new StepLoggerObserver());

        // If checked, initialize data log
        if(logSwitch.isChecked()) {
            File rawLogFile = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + getString(R.string.app_name) + "/" + RAW_LOG_FILE_NAME);
            try {
                BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(rawLogFile)));
                builder.setDataLogger(new Logger(writer));
                builder.setWifiFingerprintLogger(new PositionLogger(writer));
                builder.setMMFingerprintLogger(new PositionLogger(writer));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        // Make navigator
        IndoorNavigator nav = builder.create();
        nav.start();

        return nav;
    }
}
