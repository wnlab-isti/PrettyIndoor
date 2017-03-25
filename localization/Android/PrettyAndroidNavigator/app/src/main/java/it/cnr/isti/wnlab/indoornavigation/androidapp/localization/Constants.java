package it.cnr.isti.wnlab.indoornavigation.androidapp.localization;

import android.os.Environment;

import it.cnr.isti.wnlab.indoornavigation.R;

public final class Constants {

    private Constants() {}

    /*
     * App
     */

    public static final String APP_NAME = "PrettyIndoorNavigator";
    public static final String APP_FOLDER_PATH_EXTERNAL_STORAGE = Environment.getExternalStorageDirectory() + "/" + APP_NAME;

    /*
     * Shared Preferences
     */

    public static final String SP_NAME = "PRETTYSP";
    public static final String SP_WIFIFP_KEY = "WIFIFINGMAP";
    public static final String SP_WIFIFP_DEFAULT = Environment.getExternalStorageDirectory() + "/fingerprints/wifi_fingerprints.csv";
    public static final String SP_MAGNETIC_KEY = "MAGNETICFINGMAP";
    public static final String SP_MAGNETIC_DEFAULT = Environment.getExternalStorageDirectory() + "/fingerprints/magnetic_fingerprints.csv";
    public static final String SP_LOG_FOLDER_KEY = "LOG";
    public static final String SP_LOG_DEFAULT = APP_FOLDER_PATH_EXTERNAL_STORAGE + "/log";

    /*
     * Localization
     */

    public static final float INITIAL_X = 16.2f;
    public static final float INITIAL_Y = 5.4f;
    public static final int INITIAL_FLOOR = 1;

    /*
     * PDR
     */

    public static final float PDR_INITIAL_HEADING = 0.f;
    public static final float PDR_STEP_LENGTH = 0.6f;

    /*
     * Particle Filter
     */

    public static final int PF_PARTICLES_NUMBER = 200;

    /*
     * Wifi fingerprinting
     */
    public static final int WIFI_DISTANCES_K = 3;
    public static final float WIFI_DISTANCES_THRESHOLD = Float.MAX_VALUE;
    public static final int MAGNETIC_DISTANCES_K = 3;
    public static final float MAGNETIC_DISTANCES_THRESHOLD = Float.MAX_VALUE;
}
