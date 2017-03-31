package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.content.SharedPreferences;
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

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;

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