package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.os.Environment;

import it.cnr.isti.wnlab.indoornavigation.R;

public final class Constants {

    private Constants() {}

    /*
     * App
     */

    public static String APP_FOLDER_PATH_EXTERNAL_STORAGE = Environment.getExternalStorageDirectory() + "/" + R.string.app_name;

    /*
     * Shared Preferences
     */

    public static final String SP_NAME = "PRETTYSP";
    public static final String SP_WIFIFP_KEY = "WIFIFINGMAP";
    public static final String SP_WIFIFP_DEFAULT = Environment.getExternalStorageDirectory() + "/fingerprints/wifi_fingerprints.csv";
    public static final String SP_MAGNETIC_KEY = "MAGNETICFINGMAP";
    public static final String SP_MAGNETIC_DEFAULT = Environment.getExternalStorageDirectory() + "/fingerprints/magnetic_fingerprints.csv";

    /*
     * Localization
     */

    public static final float INITIAL_X = 19.5f;
    public static final float INITIAL_Y = 5.7f;
    public static final int INITIAL_FLOOR = 0;

    /*
     * PDR
     */

    public static final float PDR_INITIAL_HEADING = 0.f;
    public static final int PDR_COMPASS_RATE = 60;

    /*
     * Particle Filter
     */

    public static final int PF_DEFAULT_PARTICLES_NUMBER = 200;
    public static final float PF_DEFAULT_WIFI_THRESHOLD = 1000000.f;
    public static final float PF_DEFAULT_MAGNETIC_THRESHOLD = 1000000.f;
}
