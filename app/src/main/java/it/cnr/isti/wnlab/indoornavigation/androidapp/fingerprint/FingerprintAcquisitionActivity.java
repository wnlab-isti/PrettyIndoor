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
import it.cnr.isti.wnlab.indoornavigation.emitters.Emitter;
import it.cnr.isti.wnlab.indoornavigation.observers.Observer;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

public class FingerprintAcquisitionActivity extends AppCompatActivity implements View.OnClickListener {

    // Observers map
    private Map<Emitter, File> mEmitters;

    // Closeables
    private Collection<Closeable> mWriters;

    // Data structures for acquisition
    private ExecutorService mExecutorService;
    private Handler.Callback mCallback;
    private Button mButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_magnetic_fingerprint);

        // Initialization
        mEmitters = new HashMap<>();
        mWriters = new ArrayList<>();
        populateMap();

        // Register logger observer (I would like to use BiConsumer, but I can't)
        Iterator it = mEmitters.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Emitter,File> acquisition = (Map.Entry)it.next();
            try {
                mWriters.add(registerFingerprintAcquisition(acquisition));
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(),
                        "Error while opening " + acquisition.getValue().toString(),
                        Toast.LENGTH_SHORT);
            }
        }

        // GUI
        mButton = (Button) findViewById(R.id.btn_start_fingerprint);
        mButton.setOnClickListener(this);
        // Callback for reactivating button
        mCallback = new Handler.Callback() {
            @Override
            public boolean handleMessage(Message message) {
                mButton.setVisibility(View.VISIBLE);
                Toast.makeText(getApplicationContext(), "Point registered (5seconds)", Toast.LENGTH_SHORT).show();
                return true;
            }
        };

        // Thread
        mExecutorService = Executors.newFixedThreadPool(1);
    }

    private void populateMap() {
        // Fingerprints folder path
        String fingerprintFolder = Environment.getExternalStorageDirectory() + "/fingerprints";

        // Wifi initialization
        mEmitters.put(
                new WifiScanner((WifiManager) getSystemService(WIFI_SERVICE), WifiScanner.DEFAULT_SCANNING_RATE),
                new File(fingerprintFolder + "/wifi_fingerprint.csv"));

        // MF initialization
        mEmitters.put(
                new MagneticFieldHandler((SensorManager) getSystemService(SENSOR_SERVICE), SensorManager.SENSOR_DELAY_NORMAL),
                new File(fingerprintFolder + "/magnetic_fingerprint.csv"));
    }

    private Closeable registerFingerprintAcquisition(Map.Entry<Emitter,File> acq) throws IOException {
        // Get values from entry
        Emitter emitter = acq.getKey();
        File file = acq.getValue();

        // Create a writer for fingerprint acquisition
        final BufferedWriter writer = new BufferedWriter(new FileWriter(file));

        // Register
        emitter.register(new Observer<RawData>() {
            @Override
            public void notify(RawData data) {
                try {
                    writer.write(data + "\n");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        // Return writer for closing it when all is finished
        return writer;
    }

    @Override
    public void onClick(View view) {
        Toast.makeText(getApplicationContext(), "Acquisition started", Toast.LENGTH_SHORT).show();
        mButton.setVisibility(View.INVISIBLE);
        mExecutorService.submit(new AcquisitionTask(mEmitters.keySet(), mCallback));
    }

    public void onDestroy() {
        super.onDestroy();
        for(Closeable w : mWriters)
            try {
                    w.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
    }
}
