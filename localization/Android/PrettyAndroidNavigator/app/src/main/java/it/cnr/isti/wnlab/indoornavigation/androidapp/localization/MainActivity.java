package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.IBinder;
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

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;

public class MainActivity extends AppCompatActivity implements
        CompoundButton.OnCheckedChangeListener,
        View.OnClickListener {

    /*
     * Activity lifecycle
     */

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initializeGUI();

        // Set GUI mode
        if(!SimpleIndoorService.active)
            inactiveModeGUI();
        else
            activeModeGUI();

        // Set listeners
        setListeners();
    }

    @Override
    public void onResume() {
        super.onResume();

        if(SimpleIndoorService.active)
            bindToService();
    }

    @Override
    public void onPause() {
        super.onPause();

        // Set listeners
        unsetListeners();

        // Unbind
        unbindToService();
    }

    /***********************************
     * GUI
     ***********************************/

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

    /**
     * Initializes views and listeners.
     */
    private void initializeGUI() {

        // Initialize views references

        // Radio buttons
        radioFusionFilters = (RadioGroup) findViewById(R.id.radiogroup_fusionfilters);
        radioFusionFilters.check(R.id.radio_particlefilter);

        // ToggleButtons
        accToggle = (ToggleButton) findViewById(R.id.toggle_acc);
        gyroToggle = (ToggleButton) findViewById(R.id.toggle_gyro);
        magToggle = (ToggleButton) findViewById(R.id.toggle_mag);
        wifiToggle = (ToggleButton) findViewById(R.id.toggle_wifi);

        // EditText
        positionEditText = (EditText) findViewById(R.id.et_initial_position);
        positionEditText.setText(Constants.INITIAL_X + "," + Constants.INITIAL_Y);

        // Floating Action Buttons
        startFab = (FloatingActionButton) findViewById(R.id.fab_start);
        stopFab = (FloatingActionButton) findViewById(R.id.fab_stop);
        logStepFab = (FloatingActionButton) findViewById(R.id.fab_logstep);
    }
    
    private void setListeners() {
        accToggle.setOnCheckedChangeListener(this);
        gyroToggle.setOnCheckedChangeListener(this);
        magToggle.setOnCheckedChangeListener(this);
        wifiToggle.setOnCheckedChangeListener(this);

        startFab.setOnClickListener(this);
        stopFab.setOnClickListener(this);
        logStepFab.setOnClickListener(this);
    }
    
    private void unsetListeners() {
        accToggle.setOnCheckedChangeListener(null);
        gyroToggle.setOnCheckedChangeListener(null);
        magToggle.setOnCheckedChangeListener(null);
        wifiToggle.setOnCheckedChangeListener(null);

        startFab.setOnClickListener(null);
        stopFab.setOnClickListener(null);
        logStepFab.setOnClickListener(null);
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
        gyroToggle.setChecked(false);
        magToggle.setChecked(false);
        wifiToggle.setChecked(false);
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {

            case R.id.fab_start:
                try {
                    // Start service
                    startLocalizationService();
                    // Start logging
                    startLogging();
                    // Change GUI
                    activeModeGUI();
                } catch(NumberFormatException e) {
                    // EditText text is not well formatted
                    Toast.makeText(this, getString(R.string.invalid_position), Toast.LENGTH_SHORT).show();
                }
                break;

            case R.id.fab_stop:
                // Stop localization
                stopLocalizationService();
                // Stop logging
                stopLogging();
                break;

            case R.id.fab_logstep:
                if(mBoundService != null && mWriter != null)
                    logPosition(mBoundService.getPosition());
                else
                    Toast.makeText(this, getString(R.string.log_error), Toast.LENGTH_SHORT).show();
                break;
        }
    }

    private XYPosition getPositionFromEditText() throws NumberFormatException {
        // Parse EditText content
        String[] split = positionEditText.getText().toString().split(",");
        float x = Float.parseFloat(split[0]);
        float y = Float.parseFloat(split[1]);

        // No exceptions: return
        return new XYPosition(x,y);
    }

    private SimpleIndoorService.Strategies getLocalizationStrategyChoice() {
        int id = radioFusionFilters.getCheckedRadioButtonId();
        switch(id) {
            case R.id.radio_pdr:
                return SimpleIndoorService.Strategies.PDR_STRATEGY;
            case R.id.radio_wififp:
                return SimpleIndoorService.Strategies.WIFIFP_STRATEGY;
            case R.id.radio_magfp:
                return SimpleIndoorService.Strategies.MAGFP_STRATEGY;
            case R.id.radio_kalmanfilter:
                return SimpleIndoorService.Strategies.KF_STRATEGY;
            case R.id.radio_particlefilter:
                return SimpleIndoorService.Strategies.PF_STRATEGY;
        }
        return null;
    }

    @Override
    public void onCheckedChanged(CompoundButton compoundButton, boolean b) {

    }

    /***********************************
     * Logging
     ***********************************/

    private BufferedWriter mWriter;

    /**
     * Initialize objects for logging.
     */
    private void startLogging() {
        // Step logger (FAB)
        String logFolderPath = Constants.LOG_FOLDER_PATH;
        (new File(logFolderPath)).mkdirs();
        try {
            mWriter = new BufferedWriter(new FileWriter(new File(logFolderPath + "/log_" + System.currentTimeMillis() + "_" + getLocalizationStrategyChoice() + ".csv")));
        } catch(IOException e) {
            Toast.makeText(this, "Can't write on log file: " + e.getLocalizedMessage(), Toast.LENGTH_SHORT).show();
            e.printStackTrace();
        }
    }

    private void logPosition(IndoorPosition position) {
        if(mWriter != null) {
            try {
                mWriter.write(position.toString() + "\n");
            } catch(IOException e) {
                e.printStackTrace();
                Toast.makeText(getApplicationContext(), "Impossible to write position on file", Toast.LENGTH_SHORT).show();
            }
        }
    }

    private void stopLogging() {
        if(mWriter != null)
            try {
                mWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    /***********************************
     * Service client
     ***********************************/

    private SimpleIndoorService.SimpleBinder mBoundService;

    private void startLocalizationService() {
        Intent intent = new Intent(this, SimpleIndoorService.class);
        intent.putExtra(SimpleIndoorService.INTENT_START_POSITION, getPositionFromEditText());
        intent.putExtra(SimpleIndoorService.INTENT_STRATEGY_CHOICE, getLocalizationStrategyChoice());
        startService(intent);
    }

    private void stopLocalizationService() {
        stopService(new Intent(this, SimpleIndoorService.class));
    }

    private ServiceConnection mConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder service) {
            mBoundService = ((SimpleIndoorService.SimpleBinder)service).getService();
        }

        public void onServiceDisconnected(ComponentName className) {
            mBoundService = null;
        }
    };

    void bindToService() {
        bindService(new Intent(this, SimpleIndoorService.class), mConnection, BIND_IMPORTANT);

    }

    void unbindToService() {
        if (mBoundService != null)
            // Detach our existing connection.
            unbindService(mConnection);
    }

}