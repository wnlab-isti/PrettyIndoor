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
import it.cnr.isti.wnlab.indoornavigation.utils.localization.fingerprint.FingerprintStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.kalmanfilter.KalmanFilterStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.pdr.FixedStepPDR;
import it.cnr.isti.wnlab.indoornavigation.javaonly.pdr.PDR;
import it.cnr.isti.wnlab.indoornavigation.utils.localization.particlefilter.IndoorParticleFilterStrategy;

public class MainActivity extends AppCompatActivity implements
        CompoundButton.OnCheckedChangeListener,
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

    // TextViews
    private TextView wifiPositionTextView;
    private TextView magneticPositionTextView;

    // Floating Action Buttons
    private FloatingActionButton startFab;
    private FloatingActionButton stopFab;
    private FloatingActionButton logStepFab;

    /*
     * Activity lifecycle
     */

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initializeGUI();
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

    @Override
    public void onClick(View view) {
        switch(view.getId()) {

            case R.id.fab_start:
                try {
                    // Change GUI
                    activeModeGUI();

                    // Find heading-zero and startEmission localizing
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
    }

    private void inactiveModeGUI() {
        // Set "Start" and "Settings" as visible
        startFab.setVisibility(View.VISIBLE);
        stopFab.setVisibility(View.GONE);
        logStepFab.setVisibility(View.GONE);

        // Set ToggleButtons as inactive
        accToggle.setChecked(false);
        gyroToggle.setActivated(false);
        gyroToggle.setChecked(false);
        magToggle.setActivated(false);
        magToggle.setChecked(false);
        wifiToggle.setActivated(false);
        wifiToggle.setChecked(false);
    }

}