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

public class FingerprintAcquisitionActivity extends AppCompatActivity implements View.OnClickListener {

    // Observers map
    private Map<DataEmitter, File> mEmitters;

    // Writers
    private Collection<BufferedWriter> mWriters;

    // Data structures for acquisition
    private ExecutorService mExecutorService;
    private Handler.Callback mCallback;

    // GUI
    private Button mStartButton;
    private Button mUpButton;
    private Button mDownButton;
    private Button mLeftButton;
    private Button mRightButton;
    private TextView mViewX;
    private TextView mViewY;

    // Coordinates
    private float x = 0;
    private float y = 0;
    private final static float UNIT = 0.6f;

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
        (mUpButton = (Button) findViewById(R.id.btn_up)).setOnClickListener(this);
        (mDownButton = (Button) findViewById(R.id.btn_down)).setOnClickListener(this);
        (mLeftButton = (Button) findViewById(R.id.btn_left)).setOnClickListener(this);
        (mRightButton = (Button) findViewById(R.id.btn_right)).setOnClickListener(this);

        // TextViews
        mViewX = (TextView) findViewById(R.id.tv_x);
        mViewY = (TextView) findViewById(R.id.tv_y);
        mViewX.setText(x + "");
        mViewY.setText(y + "");

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
        // Fingerprints folder path (create if it doesn't exist)
        String fingerprintFolderPath = Environment.getExternalStorageDirectory() + "/fingerprints";
        File fingerprintsFolder = new File(fingerprintFolderPath);
        fingerprintsFolder.mkdirs();

        // Current timestamp (for unique files)
        Long timestamp = System.currentTimeMillis();

        // Wifi initialization
        mEmitters.put(
                new WifiScanner((WifiManager) getSystemService(WIFI_SERVICE), WifiScanner.DEFAULT_SCANNING_RATE),
                new File(fingerprintFolderPath + "/fingerprint_wifi_" + timestamp + ".csv"));

        // MF initialization
        mEmitters.put(
                new MagneticFieldHandler((SensorManager) getSystemService(SENSOR_SERVICE), SensorManager.SENSOR_DELAY_FASTEST),
                new File(fingerprintFolderPath + "/fingerprint_magnetic_" + timestamp + " .csv"));

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
            case R.id.btn_start_fingerprint:
                // Change button's visibility
                mStartButton.setVisibility(View.INVISIBLE);

                // Do everything you have to do right before writing data and start the task
                prepareNewPoint();
                startAcquisition();
                mExecutorService.submit(new AcquisitionTask(mCallback));

                // Signal start
                Toast.makeText(getApplicationContext(), "Acquisition started", Toast.LENGTH_SHORT).show();
                break;

            case R.id.btn_left:
                x -= UNIT;
                mViewX.setText(x + "");
                break;

            case R.id.btn_right:
                x += UNIT;
                mViewX.setText(x + "");
                break;

            case R.id.btn_up:
                y += UNIT;
                mViewY.setText(y + "");
                break;

            case R.id.btn_down:
                y -= UNIT;
                mViewY.setText(y + "");
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
                w.write(x + "\t" + y + "\n");
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "Impossible flushing " + e.getLocalizedMessage(),  Toast.LENGTH_LONG).show();
            }
    }

    private void startAcquisition() {
        for(DataEmitter e : mEmitters.keySet())
            e.start();
    }

    private void stopAcquisition() {
        for(DataEmitter e : mEmitters.keySet())
            e.stop();
    }
}
