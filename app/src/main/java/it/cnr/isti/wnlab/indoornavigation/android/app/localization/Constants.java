package it.cnr.isti.wnlab.indoornavigation.android.app.localization;

import android.os.Environment;

/**
 * Application's configuration.
 */
public final class Constants {

    private Constants() {}

    /*
     * App
     */

    public static final String APP_NAME = "PrettyIndoorNavigator";
    public static final String APP_FOLDER_PATH_EXTERNAL_STORAGE = Environment.getExternalStorageDirectory() + "/" + APP_NAME;
    public static final String FP_FOLDER_PATH = Environment.getExternalStorageDirectory() + "/fingerprints";
    public static final String WIFIFP_DEFAULT = FP_FOLDER_PATH + "/wifi_fingerprints.csv";
    public static final String MAGFP_DEFAULT = FP_FOLDER_PATH + "/magnetic_fingerprints.csv";
    public static final String LOG_FOLDER_PATH = APP_FOLDER_PATH_EXTERNAL_STORAGE + "/log";

    /*
     * Wifi
     */

    public static final long WIFI_RATE = 1400;

    /*
     * Localization
     */

    public static final float INITIAL_X = 16.2f;
    public static final float INITIAL_Y = 5.4f;

    /*
     * PDR
     */

    public static final float PDR_INITIAL_HEADING = 0.f;
    public static final float PDR_STEP_LENGTH = 0.6f;

    /*
     * Kalman StateEstimationFilter
     */

    public static final float KF_WIFI_POSITION_RADIUS = 5.f;
    public static final int KF_WIFI_DISTANCES_K = 3;
    public static final int KF_MAGNETIC_DISTANCES_K = 9;

    /*
     * Particle StateEstimationFilter
     */

    public static final int PF_PARTICLES_NUMBER = 200;
    public static final int PF_WIFI_DISTANCES_K = 3;
    public static final int PF_MAGNETIC_DISTANCES_K = 3;

}
