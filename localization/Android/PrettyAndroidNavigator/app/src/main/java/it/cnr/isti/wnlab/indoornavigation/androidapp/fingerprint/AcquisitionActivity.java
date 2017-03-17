package it.cnr.isti.wnlab.indoornavigation.androidapp.fingerprint;

import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.File;
import java.io.FileWriter;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.MagneticFieldHandler;
import it.cnr.isti.wnlab.indoornavigation.android.wifi.WifiScanner;
import it.cnr.isti.wnlab.indoornavigation.androidapp.Logger;
import it.cnr.isti.wnlab.indoornavigation.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.observer.Emitter;

public class AcquisitionActivity extends AppCompatActivity implements View.OnClickListener {

    // Observers map
    private Map<DataEmitter, File> mEmitters;

    // Writers
    private Collection<BufferedWriter> mWriters;
    private boolean first = true;

    // Data structures for acquisition
    private ExecutorService mExecutorService;
    private Handler.Callback mCallback;

    // GUI
    private Button mStartButton;
    private TextView mViewX;
    private TextView mViewY;

    // Coordinates in MILLIMETERS
    private int y = 5400;
    private int x = 6000;
    private final static int STEP = 600;
    private final static float DIVISOR = 1000.f;

    // Folder path constants
    public static final String FINGERPRINT_FOLDER = Environment.getExternalStorageDirectory() + "/fingerprints";
    public static final String MAGNETIC_DATA_FOLDER = FINGERPRINT_FOLDER + "/magnetic";
    public static final String WIFI_DATA_FOLDER = FINGERPRINT_FOLDER + "/wifi";

    // Data file prefixes
    public static final String WIFI_DATA_FILE_PREFIX = "wifi_";
    public static final String MAGNETIC_DATA_FILE_PREFIX = "magnetic_";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_fingerprint);

        // Initialization
        mEmitters = new HashMap<>();
        mWriters = new ArrayList<>();
        populateMap();

        // GUI

        // Start button
        mStartButton = (Button) findViewById(R.id.btn_start_fingerprint);
        mStartButton.setOnClickListener(this);

        // Directional buttons
        (findViewById(R.id.btn_up)).setOnClickListener(this);
        (findViewById(R.id.btn_down)).setOnClickListener(this);
        (findViewById(R.id.btn_left)).setOnClickListener(this);
        (findViewById(R.id.btn_right)).setOnClickListener(this);
        (findViewById(R.id.btn_make_magnetic)).setOnClickListener(this);
        (findViewById(R.id.btn_make_wifi)).setOnClickListener(this);

        // TextViews
        mViewX = (TextView) findViewById(R.id.tv_x);
        mViewY = (TextView) findViewById(R.id.tv_y);
        mViewX.setText("x: " + x/DIVISOR);
        mViewY.setText("y: " + y/DIVISOR);

        // Callback for reactivating button
        mCallback = new Handler.Callback() {
            @Override
            public boolean handleMessage(Message message) {
                // Stop acquisition
                stopAcquisition();
                // Reactivate start button
                mStartButton.setVisibility(View.VISIBLE);
                // Signal acquisition finish to the user
                Toast.makeText(getApplicationContext(), "Point registered (5seconds)", Toast.LENGTH_SHORT).show();

                return true;
            }
        };

        // Thread
        mExecutorService = Executors.newFixedThreadPool(1);
    }

    /**
     * Initialize data structures for the data we want to register.
     */
    private void populateMap() {
        // Make folders (if needed)
        File fingerprintsFolder = new File(FINGERPRINT_FOLDER);
        fingerprintsFolder.mkdirs();
        File wifiDataFolder = new File(WIFI_DATA_FOLDER);
        wifiDataFolder.mkdir();
        File magneticDataFolder = new File(MAGNETIC_DATA_FOLDER);
        magneticDataFolder.mkdir();
    }

    /**
     * Flush writers onStop.
     */
    public void onStop() {
        super.onStop();
        for(BufferedWriter w : mWriters)
            try {
                w.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    /**
     * OnDestroy close the closeables.
     */
    public void onDestroy() {
        super.onDestroy();
        for(Closeable w : mWriters)
            try {
                w.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    /**
     * onClick listener.
     * @param view
     */
    @Override
    public void onClick(View view) {
        int id = view.getId();

        switch(id) {
            /**
             * Button for start the data acquisition.
             */
            case R.id.btn_start_fingerprint:
                // Initialize writers before first write
                if(first) {
                    // Current timestamp (for unique files)
                    Long timestamp = System.currentTimeMillis();

                    // Wifi initialization
                    mEmitters.put(
                            new WifiScanner((WifiManager) getSystemService(WIFI_SERVICE), WifiScanner.DEFAULT_SCANNING_RATE),
                            new File(WIFI_DATA_FOLDER + "/" + WIFI_DATA_FILE_PREFIX + timestamp + ".csv"));

                    // MF initialization
                    mEmitters.put(
                            new MagneticFieldHandler((SensorManager) getSystemService(SENSOR_SERVICE), SensorManager.SENSOR_DELAY_FASTEST),
                            new File(MAGNETIC_DATA_FOLDER + "/" + MAGNETIC_DATA_FILE_PREFIX + timestamp + " .csv"));

                    // Register logger observer (I would like to use BiConsumer, but I can't)
                    Iterator it = mEmitters.entrySet().iterator();
                    while (it.hasNext()) {
                        Map.Entry<Emitter,File> acquisition = (Map.Entry)it.next();
                        try {
                            // Create a writer for fingerprint acquisition
                            BufferedWriter writer = new BufferedWriter(new FileWriter(acquisition.getValue()));

                            // Register logger and add writer to collection
                            acquisition.getKey().register(new Logger(writer));
                            mWriters.add(writer);
                        } catch (IOException e) {
                            Toast.makeText(getApplicationContext(),
                                    "Error while opening " + acquisition.getValue().toString(),
                                    Toast.LENGTH_SHORT);
                        }
                    }

                    first = false;
                }

                // Change button's visibility
                mStartButton.setVisibility(View.INVISIBLE);

                // Do everything you have to do right before writing data and start the task
                prepareNewPoint();
                startAcquisition();
                mExecutorService.submit(new AcquisitionTask(mCallback));

                // Signal start
                Toast.makeText(getApplicationContext(), "Acquisition started", Toast.LENGTH_SHORT).show();
                break;

            /**
             * Buttons for moving.
             */
            case R.id.btn_left:
                x -= STEP;
                //COORDINATE_FORMAT.format(x);
                mViewX.setText("x: " + x/DIVISOR);
                break;

            case R.id.btn_right:
                x += STEP;
                //COORDINATE_FORMAT.format(x);
                mViewX.setText("x: " + x/DIVISOR);
                break;

            case R.id.btn_up:
                y += STEP;
                //COORDINATE_FORMAT.format(y);
                mViewY.setText("y: " + y/DIVISOR);
                break;

            case R.id.btn_down:
                y -= STEP;
                //COORDINATE_FORMAT.format(y);
                mViewY.setText("y: " + y/DIVISOR);
                break;

            /**
             * Button for making the magnetic fingerprint.
             */
            case R.id.btn_make_magnetic:
                Toast.makeText(getApplicationContext(), "Starting fingerprint creation", Toast.LENGTH_SHORT).show();

                // Initialize fingerprint builder
                MagneticFingerprintFileBuilder magneticBuilder = new MagneticFingerprintFileBuilder();

                // Get data files from default folder
                File[] magDataFiles = getDataFiles(MAGNETIC_DATA_FILE_PREFIX, new File(MAGNETIC_DATA_FOLDER));

                // Check for magnetic data files
                if(magDataFiles != null) {
                    Toast.makeText(getApplicationContext(),"Found " + magDataFiles.length + " files with Wifi data", Toast.LENGTH_SHORT).show();

                    // The resulting file the fingerprint has to be saved in
                    File result = new File(FINGERPRINT_FOLDER + "/magnetic_fingerprint.csv");

                    // Make fingerprint
                    try {
                        magneticBuilder.make(result, magDataFiles);
                    } catch(IOException e) {
                        e.printStackTrace();
                    }

                    Toast.makeText(getApplicationContext(), "Fingerprint made!", Toast.LENGTH_SHORT).show();
                } else
                    Toast.makeText(getApplicationContext(), "No data files found for magnetic field.", Toast.LENGTH_SHORT);
                break;

            /**
             * Button for making the wifi fingerprint.
             */
            case R.id.btn_make_wifi:
                Toast.makeText(getApplicationContext(), "Starting fingerprint creation", Toast.LENGTH_SHORT).show();

                // Initialize fingerprint builder
                WifiFingerprintFileBuilder wifiBuilder = new WifiFingerprintFileBuilder();

                // Get data files from default folder
                File[] wifiDataFiles = getDataFiles(WIFI_DATA_FILE_PREFIX, new File(WIFI_DATA_FOLDER));

                // Check for magnetic data files
                if(wifiDataFiles != null) {
                    Toast.makeText(getApplicationContext(),"Found " + wifiDataFiles.length + " files with Wifi data", Toast.LENGTH_SHORT).show();

                    // The resulting file the fingerprint has to be saved in
                    File result = new File(FINGERPRINT_FOLDER + "/wifi_fingerprint.csv");

                    // Make fingerprint
                    try {
                        wifiBuilder.make(result, wifiDataFiles);
                    } catch (IOException e) {
                        e.printStackTrace();
                    }

                    Toast.makeText(getApplicationContext(), "Fingerprint made!", Toast.LENGTH_SHORT).show();
                } else
                    Toast.makeText(getApplicationContext(), "No data files found for wifi.", Toast.LENGTH_SHORT);
                break;
        }
    }

    /**
     * Flush all writers.
     */
    private void prepareNewPoint() {
        // Flush all writers and write new point coordinates
        for(BufferedWriter w : mWriters)
            try {
                w.flush();
                w.write(x/DIVISOR + "\t" + y/DIVISOR + "\n");
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "Impossible flushing " + e.getLocalizedMessage(),  Toast.LENGTH_LONG).show();
            }
    }

    /**
     * Starts acquisition for current point.
     */
    private void startAcquisition() {
        for(DataEmitter e : mEmitters.keySet())
            e.start();
    }

    /**
     * Stops acquisition for current point.
     */
    private void stopAcquisition() {
        for(DataEmitter e : mEmitters.keySet())
            e.stop();
    }

    /**
     * @param prefix The prefix for specified data files.
     * @param directory The directory that contains data files.
     * @return The data files filtered by prefix.
     */
    private File[] getDataFiles(final String prefix, File directory) {
        return directory.listFiles(new FilenameFilter() {
            @Override
            public boolean accept(File file, String s) {
                return s.startsWith(prefix);
            }
        });
    }
}
