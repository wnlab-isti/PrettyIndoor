package it.cnr.isti.wnlab.indoornavigation;

import it.cnr.isti.wnlab.indoornavigation.observers.Observer;

public interface IndoorNavigator extends StartableStoppable {
    // Position Updater getter and setter
    void setPositionUpdater(Observer<IndoorPosition> updater);
    Observer<IndoorPosition> getPositionUpdater();
}
