package it.cnr.isti.wnlab.indoornavigator.android;


import java.io.IOException;
import java.io.Writer;

import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.types.Acceleration;
import it.cnr.isti.wnlab.indoornavigator.framework.types.MagneticField;
import it.cnr.isti.wnlab.indoornavigator.framework.types.Rotation;
import it.cnr.isti.wnlab.indoornavigator.framework.types.WifiFingerprint;

/**
 * Logs on a file in external storage download directory.
 */
public class Logger {

    private Logger() {}

    /**
     * Log with StepLogger's format.
     */
    public static void steplog(Writer writer, IndoorPosition position)
            throws IOException {
        writer.write(position.timestamp + " " + position.x + " " + position.y + " " + position.floor + "\n");
    }

    /**
     * General format for sensor logging.
     * @param writer
     * @param tag
     * @param log
     * @throws IOException
     */
    private static void sensorLog(Writer writer, String tag, String log) throws IOException {
        writer.write(tag + log + "\n");
    }

    public static void logAccelerometer(Writer writer, Acceleration a) throws IOException {
        sensorLog(writer,"A",a.toString());
    }

    public static void logGyroscope(Writer writer, Rotation r) throws IOException {
        sensorLog(writer,"G",r.toString());
    }

    public static void logMagnetometer(Writer writer, MagneticField mf) throws IOException {
        sensorLog(writer,"M",mf.toString());
    }

    public static void logWifiFingerprint(Writer writer, WifiFingerprint fingerprint) throws IOException {
        sensorLog(writer,"W",fingerprint.toString());
    }
}