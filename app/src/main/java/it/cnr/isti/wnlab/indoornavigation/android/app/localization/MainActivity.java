package it.cnr.isti.wnlab.indoornavigation.android.app.localization;

import android.Manifest;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.IBinder;
import android.support.design.widget.FloatingActionButton;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.RadioGroup;
import android.widget.Toast;
import android.widget.ToggleButton;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.XYPosition;

/**
 * In MVC pattern-view here are the view and some controllers. The Model is actually in the service.
 * Here is the code of the main activity life-cycle's, GUI, logging and client-side service.
 */
public class MainActivity extends AppCompatActivity implements View.OnClickListener {

    /***********************************
     * User's permission request
     ***********************************/

    private final int PERMISSION_CODE_ALL = 1;

    private void checkForPermissions(String... permissions) {
        for(String permission : permissions)
            // Here, thisActivity is the current activity
            if (ContextCompat.checkSelfPermission(this,
                    permission)
                    != PackageManager.PERMISSION_GRANTED) {

                // It should be more friendly checking if user needs explanations
                ActivityCompat.requestPermissions(this,
                        new String[]{permission},
                        PERMISSION_CODE_ALL);
            }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String permissions[], int[] grantResults) {
        switch (requestCode) {
            case PERMISSION_CODE_ALL: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                    // Everything's ok

                } else {

                    // We have not all the permissions! Brutally close everything
                    Toast.makeText(this, getResources().getString(R.string.no_permissions), Toast.LENGTH_LONG).show();
                    finish();

                }

            }
        }
    }

    /***********************************
     * Activity lifecycle
     ***********************************/

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Asynchronously ask runtime permissions
        checkForPermissions(
                Manifest.permission.READ_EXTERNAL_STORAGE,
                Manifest.permission.WRITE_EXTERNAL_STORAGE,
                Manifest.permission.ACCESS_FINE_LOCATION);

        // Initialize GUI elements
        initializeGUI();
    }

    @Override
    public void onResume() {
        super.onResume();

        // Set GUI mode
        if(!SimpleIndoorService.active)
            inactiveModeGUI();
        else
            activeModeGUI();

        // Set listeners
        setListeners();

        // Bind to service if it is active
        if(SimpleIndoorService.active && mBoundService == null)
            bindToService();
    }

    @Override
    public void onPause() {
        super.onPause();

        // Set listeners
        unsetListeners();
    }

    @Override
    public void onStop() {
        super.onStop();

        // Unbind
        if(SimpleIndoorService.active)
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
        startFab.setOnClickListener(this);
        stopFab.setOnClickListener(this);
        logStepFab.setOnClickListener(this);
    }
    
    private void unsetListeners() {
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
                    bindToService();
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
                unbindToService();
                stopLocalizationService();
                // Stop logging
                stopLogging();
                // Change GUI
                inactiveModeGUI();
                break;

            case R.id.fab_logstep:
                Log.d("STR", "bound service: " + mBoundService);
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
        Log.d("STR", id + "");
        switch(id) {
            case R.id.radio_pdr:
                accToggle.setChecked(true);
                magToggle.setChecked(true);
                gyroToggle.setChecked(true);
                wifiToggle.setChecked(false);
                return SimpleIndoorService.Strategies.PDR_STRATEGY;
            case R.id.radio_wififp:
                accToggle.setChecked(false);
                magToggle.setChecked(false);
                gyroToggle.setChecked(false);
                wifiToggle.setChecked(true);
                return SimpleIndoorService.Strategies.WIFIFP_STRATEGY;
            case R.id.radio_magfp:
                accToggle.setChecked(false);
                magToggle.setChecked(true);
                gyroToggle.setChecked(false);
                wifiToggle.setChecked(false);
                return SimpleIndoorService.Strategies.MAGFP_STRATEGY;
            case R.id.radio_kalmanfilter:
                accToggle.setChecked(true);
                magToggle.setChecked(true);
                gyroToggle.setChecked(true);
                wifiToggle.setChecked(true);
                return SimpleIndoorService.Strategies.KF_STRATEGY;
            case R.id.radio_particlefilter:
                accToggle.setChecked(true);
                magToggle.setChecked(true);
                gyroToggle.setChecked(true);
                wifiToggle.setChecked(true);
                return SimpleIndoorService.Strategies.PF_STRATEGY;
        }
        throw new RuntimeException("No strategy is specified.");
    }

    /***********************************
     * Logging
     ***********************************/

    private BufferedWriter mWriter;

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
                Log.d("STR","Position written: " + position);
            } catch(IOException e) {
                e.printStackTrace();
                Toast.makeText(getApplicationContext(), "Impossible to write position on file", Toast.LENGTH_SHORT).show();
            }
        }
    }

    private void stopLogging() {
        if(mWriter != null)
            try {
                mWriter.flush();
                mWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    /***********************************
     * Service client
     ***********************************/

    private SimpleIndoorService.SimpleBinder mBoundService;
    private ServiceConnection mConnection;

    private void startLocalizationService() {
        Intent intent = new Intent(this, SimpleIndoorService.class);
        intent.putExtra(SimpleIndoorService.INTENT_START_POSITION, getPositionFromEditText());
        intent.putExtra(SimpleIndoorService.INTENT_STRATEGY_CHOICE, getLocalizationStrategyChoice());
        startService(intent);
    }

    private void stopLocalizationService() {
        stopService(new Intent(this, SimpleIndoorService.class));
    }

    void bindToService() {
        mConnection = new ServiceConnection() {
            public void onServiceConnected(ComponentName className, IBinder service) {
                mBoundService = ((SimpleIndoorService.SimpleBinder) service).getService();
            }

            public void onServiceDisconnected(ComponentName className) {
                mBoundService = null;
            }
        };
        bindService(new Intent(this, SimpleIndoorService.class), mConnection, BIND_IMPORTANT);
    }

    void unbindToService() {
        if (mBoundService != null) {
            // Detach our existing connection
            unbindService(mConnection);
            mBoundService = null;
        }
    }

}