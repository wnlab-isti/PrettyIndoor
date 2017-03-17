package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

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
import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.android.wifi.WifiScanner;
import it.cnr.isti.wnlab.indoornavigation.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.GeomagneticFingerprint;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.WifiFingerprint;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.AccessPoints;

public class FingerprintTestActivity extends AppCompatActivity {

    private WifiScanner wifiScanner;

    private GeomagneticFingerprint mfing;
    private WifiFingerprint wifing;

    private String wifiFingPath = Environment.getExternalStorageDirectory() + "/fingerprints/wifi_fingerprint.csv";

    private TextView textView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_fingerprint_test);

        textView = (TextView) (findViewById(R.id.tv_positions));

        // Wifi

        // Wifi Scanner
        wifiScanner = new WifiScanner((WifiManager) getSystemService(WIFI_SERVICE));

        // Load Fingerprint button
        (findViewById(R.id.fp_test_btn_load_wifi)).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                WifiFingerprint.Builder wifiBuilder = new WifiFingerprint.Builder();
                wifing = wifiBuilder.buildFromFile(new File(wifiFingPath));
            }
        });

        // Toggle wifi scansion
        ((ToggleButton) findViewById(R.id.fp_test_toggle_scanning_wifi)).setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if(b)
                    wifiScanner.start();
                else
                    wifiScanner.stop();
            }
        });

        (findViewById(R.id.fp_test_btn_localize_wifi)).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(),"Start listening to scansions",Toast.LENGTH_SHORT).show();

                DataObserver<AccessPoints> observer = new DataObserver<AccessPoints>() {
                    @Override
                    public void notify(AccessPoints data) {
                        int k = 3;

                        Toast.makeText(getApplicationContext(), "Doing K-NN", Toast.LENGTH_SHORT).show();
                        List<XYPosition> positions = wifing.findNearestK(data,k);
                        Toast.makeText(getApplicationContext(),positions.size() + " positions found",Toast.LENGTH_SHORT).show();
                        StringBuilder string = new StringBuilder();

                        float avgX = 0.f;
                        float avgY = 0.f;
                        for(XYPosition p : positions) {
                            string.append(p + "\n");
                            avgX += p.x;
                            avgY += p.y;
                        }
                        textView.setText(string);

                        avgX /= k;
                        avgY /= k;
                        Toast.makeText(getApplicationContext(),"Average position: " + new XYPosition(avgX,avgY),Toast.LENGTH_SHORT).show();

                        //Toast.makeText(getApplicationContext(),"Unregistering",Toast.LENGTH_SHORT).show();
                        //wifiScanner.unregister(this);
                    }
                };
                wifiScanner.register(observer);
            }
        });
    }

}
