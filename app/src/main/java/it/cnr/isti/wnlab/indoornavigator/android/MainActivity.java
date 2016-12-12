package it.cnr.isti.wnlab.indoornavigator.android;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;

import java.io.File;

public class MainActivity extends AppCompatActivity {

    private File wifiFingerprintDB;
    private File magneticFingerprintDB;

    private static final int WIFI_REQUEST_CODE = 42;
    private static final int MAGNETIC_REQUEST_CODE = 1337;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (resultCode == RESULT_OK) {
            switch(requestCode) {
                // From WifiDB file request
                case WIFI_REQUEST_CODE:
                    wifiFingerprintDB = new File(data.getData().getPath());
                    break;
                // From geomagnetic DB file request
                case MAGNETIC_REQUEST_CODE:
                    magneticFingerprintDB = new File(data.getData().getPath());
                    break;
            }
        }
    }

    private void initializeFileRequest(int requestCode) {
        Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
        intent.setType("text/plain");
        startActivityForResult(intent, requestCode);
    }


}
