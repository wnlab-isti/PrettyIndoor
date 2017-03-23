package it.cnr.isti.wnlab.indoornavigation.androidapp.fingerprint;

import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;

import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import java.io.File;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.MagnetometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.WifiScanner;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.FingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.MagneticFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.WifiFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;

public class FingerprintTestActivity extends AppCompatActivity {

    // Wifi data
    private WifiScanner wifiScanner;
    private WifiFingerprintMap wiFing;
    private String wiFingPath = Environment.getExternalStorageDirectory() + "/fingerprints/wifi_fingerprint.csv";

    // Magnetic data
    private MagnetometerHandler mHandler;
    private MagneticFingerprintMap mFing;
    private String mFingPath = Environment.getExternalStorageDirectory() + "/fingerprints/magnetic_fingerprint.csv";

    // GUI
    private TextView tvPositions;
    private String mTitle;

    // Constants
    public final static String FINGERPRINT_TYPE = "fp_type";
    public final static String TEST_WIFI = "wifi";
    public final static String TEST_MAGNETIC = "magnetic";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_fingerprint_test);

        tvPositions = (TextView) (findViewById(R.id.tv_positions));

        // Check for fingerprint type
        String testType = getIntent().getStringExtra(FINGERPRINT_TYPE);
        if(TEST_WIFI.equals(testType)) {
            // Wifi fingerprint
            initializeWifi();
            mTitle = "Wifi FP test";
        } else if(TEST_MAGNETIC.equals(testType)) {
            // Magnetic fingerprint
            initializeMagnetic();
            mTitle = "Magnetic FP test";
        } else {
            // Error
            Toast.makeText(getApplicationContext(), "Invalid fingerprint type.", Toast.LENGTH_LONG).show();
            finish();
        }

        ((TextView) findViewById(R.id.fp_test_tv_title)).setText(mTitle);
    }

    /**
     * Initialize data structures and observers for testing a magnetic fingerprint.
     */
    private void initializeWifi() {
        // Wifi Scanner
        wifiScanner = new WifiScanner((WifiManager) getSystemService(WIFI_SERVICE));

        // Load FingerprintMap button
        (findViewById(R.id.fp_test_btn_load)).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                WifiFingerprintMap.Builder wifiBuilder = new WifiFingerprintMap.Builder();
                wiFing = wifiBuilder.build(new File(wiFingPath));
            }
        });

        // Toggle wifi scansion
        ((ToggleButton) findViewById(R.id.fp_test_toggle_scanning)).setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if(b)
                    wifiScanner.start();
                else
                    wifiScanner.stop();
            }
        });

        (findViewById(R.id.fp_test_btn_localize)).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(),"Start listening to scansions",Toast.LENGTH_SHORT).show();

                DataObserver<AccessPoints> observer = new DataObserver<AccessPoints>() {
                    @Override
                    public void notify(AccessPoints data) {
                        int k = 3;
/*
                        Toast.makeText(getApplicationContext(), "Doing K-NN", Toast.LENGTH_SHORT).show();
                        List<FingerprintMap.PositionDistance> positions = wiFing.findNearestK(data,k,1000.f);
                        Toast.makeText(getApplicationContext(),positions.size() + " positions found",Toast.LENGTH_SHORT).show();
                        StringBuilder string = new StringBuilder();

                        float avgX = 0.f;
                        float avgY = 0.f;
                        for(XYPosition p : positions) {
                            string.append(p + "\n");
                            avgX += p.x;
                            avgY += p.y;
                        }
                        tvPositions.setText(string);

                        avgX /= k;
                        avgY /= k;
                        Toast.makeText(getApplicationContext(),"Average position: " + new XYPosition(avgX,avgY),Toast.LENGTH_SHORT).show();

                        //Toast.makeText(getApplicationContext(),"Unregistering",Toast.LENGTH_SHORT).show();
                        //wifiScanner.unregister(this);*/
                    }
                };
                wifiScanner.register(observer);
            }
        });
    }

    /**
     * Initialize data structures and observers for testing a magnetic fingerprint.
     */
    private void initializeMagnetic() {
        // Magnetic events listener
        int speed = 1500;
        mHandler = new MagnetometerHandler((SensorManager) getSystemService(SENSOR_SERVICE), speed);

        // Load FingerprintMap button
        (findViewById(R.id.fp_test_btn_load)).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                MagneticFingerprintMap.Builder magBuilder = new MagneticFingerprintMap.Builder();
                mFing = magBuilder.build(new File(mFingPath));
            }
        });

        // Toggle wifi scansion
        ((ToggleButton) findViewById(R.id.fp_test_toggle_scanning)).setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if(b)
                    mHandler.start();
                else
                    mHandler.stop();
            }
        });

        (findViewById(R.id.fp_test_btn_localize)).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                /*Toast.makeText(getApplicationContext(),"Start listening to scansions",Toast.LENGTH_SHORT).show();

                DataObserver<MagneticField> observer = new DataObserver<MagneticField>() {
                    @Override
                    public void notify(MagneticField data) {
                        int k = 1;

                        Toast.makeText(getApplicationContext(), "Doing K-NN", Toast.LENGTH_SHORT).show();
                        List<XYPosition> positions = mFing.findNearestK(data,k,1000.f);
                        Toast.makeText(getApplicationContext(),positions.size() + " positions found",Toast.LENGTH_SHORT).show();
                        StringBuilder string = new StringBuilder();

                        float avgX = 0.f;
                        float avgY = 0.f;
                        for(XYPosition p : positions) {
                            string.append(p + "\n");
                            avgX += p.x;
                            avgY += p.y;
                        }
                        tvPositions.setText(string);

                        avgX /= k;
                        avgY /= k;
                        Toast.makeText(getApplicationContext(),"Average position: " + new XYPosition(avgX,avgY),Toast.LENGTH_SHORT).show();

                        mHandler.unregister(this);
                    }
                };
                mHandler.register(observer);*/
            }
        });
    }

}
