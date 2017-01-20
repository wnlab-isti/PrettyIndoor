package it.cnr.isti.wnlab.indoornavigator;

import it.cnr.isti.wnlab.indoornavigator.observers.Observer;

public interface IndoorNavigator extends StartableStoppable {
    // Position Updater getter and setter
    void setPositionUpdater(Observer<IndoorPosition> updater);
    Observer<IndoorPosition> getPositionUpdater();
}
