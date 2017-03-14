package it.cnr.isti.wnlab.indoornavigation.androidapp.fingerprint;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;

public abstract class Fingerprint {

    /**
     * Creates a file with a fingerprint made from csv data files.
     * @param result The fingerprint file that has to be made.
     * @param filesWithData Files containing data.
     * @return true if the fingerprint has been correctly written, false if an error occurred.
     */
    public boolean make(File result, File... filesWithData) {
        if(filesWithData != null) {

            // For each file, add the measures to the map
            for(File f : filesWithData) {
                try (
                    BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(f)));
                ) {
                    // Get the label that starts each measure
                    String label = getLabel();

                    // For each line, add points and relative measures to the data structure
                    String line;
                    while((line = br.readLine()) != null) {
                        // Split comma-separated values
                        String[] splitted = line.split(",");

                        // Behaviour depends on the first comma-separated value
                        if(label.equals(splitted[0])) {
                            // If the line starts with the label, it's a measure
                            insertMeasure(splitted);
                        } else {
                            // Else it's a coordinate indicator
                            float x = Float.parseFloat(splitted[0]);
                            float y = Float.parseFloat(splitted[1]);
                            setCoordinates(x,y);
                        }
                    }

                    // Merge measures of the data structure
                    merge();

                    // Write fingerprint on file
                    try(
                        BufferedWriter writer = new BufferedWriter(new FileWriter(result));
                    ) {
                        writeOnFile(writer);

                        // Success
                        return true;
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            // An error occurred
            return false;

        } else
            throw new NullPointerException("You must specify at least one file for this operation.");
    }

    /**
     * @return The label the lines with measures start with. If it doesn't start with this, it
     * indicates a coordinate.
     */
    protected abstract String getLabel();

    /**
     * Sets the current coordinate in the data structure.
     * @param x
     * @param y
     */
    protected abstract void setCoordinates(float x, float y);

    /**
     * Insert a new measure in the current point's values.
     * @param commaSeparatedValues
     */
    protected abstract void insertMeasure(String[] commaSeparatedValues);

    /**
     * Merges the values for each point. This method actually prepares the fingerprint and must be
     * called before writeOnFile.
     */
    protected abstract void merge();

    /**
     * Writes the fingerprint. This method must be called after merge().
     * @param writer
     */
    protected abstract void writeOnFile(BufferedWriter writer);

}
