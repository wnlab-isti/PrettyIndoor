package it.cnr.isti.wnlab.indoornavigator.androidapp.fingerprint.geomagnetic;

import android.hardware.SensorManager;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import it.cnr.isti.wnlab.indoornavigator.R;
import it.cnr.isti.wnlab.indoornavigator.android.handlers.MagneticFieldHandler;
import it.cnr.isti.wnlab.indoornavigator.observers.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.types.environmental.MagneticField;

public class GeomagneticFingerprintActivity extends AppCompatActivity implements View.OnClickListener {

    private MagneticFieldHandler mMag;
    private BufferedWriter mWriter;
    private ExecutorService mExecutorService;
    private Handler.Callback mCallback;
    private Button mButton;
    private final static String LOG_FILE_NAME = "geomag_fingerprint.log";


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_magnetic_fingerprint);

        // MF sensor
        SensorManager manager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mMag = new MagneticFieldHandler(manager, SensorManager.SENSOR_DELAY_NORMAL);

        // Register logger observer
        mMag.register(new DataObserver<MagneticField>() {
            @Override
            public void notify(MagneticField data) {
                try {
                    mWriter.write(data + "\n");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        // Writer
        try {
            mWriter = new BufferedWriter(new FileWriter(Environment.getExternalStorageDirectory() + "/" + LOG_FILE_NAME,true));
        } catch (IOException e) {
            e.printStackTrace();
        }

        // GUI
        mButton = (Button) findViewById(R.id.btn_start_fingerprint);
        mButton.setOnClickListener(this);
        // Callback for reactivating button
        mCallback = new Handler.Callback() {
            @Override
            public boolean handleMessage(Message message) {
                mButton.setVisibility(View.VISIBLE);
                Toast.makeText(getApplicationContext(), "Point registered (5seconds passed)", Toast.LENGTH_SHORT).show();
                return true;
            }
        };

        // Thread
        mExecutorService = Executors.newFixedThreadPool(1);
    }

    @Override
    public void onClick(View view) {
        Toast.makeText(getApplicationContext(), "Acquisition started", Toast.LENGTH_SHORT).show();
        mButton.setVisibility(View.INVISIBLE);
        mExecutorService.submit(new MagneticAcquireTask(mWriter, mMag, mCallback));
    }

    public void onDestroy() {
        super.onDestroy();
        try {
            mWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
