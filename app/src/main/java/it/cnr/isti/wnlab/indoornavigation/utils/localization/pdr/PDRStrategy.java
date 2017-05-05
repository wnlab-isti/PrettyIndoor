package it.cnr.isti.wnlab.indoornavigation.utils.localization.pdr;

import it.cnr.isti.wnlab.indoornavigation.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.pdr.PDR;

/**
 * A not-so-smart localization algorithm that uses only PDR.
 */
public class PDRStrategy extends AbstractIndoorLocalizationStrategy implements Observer<PDR.Result> {

    private XYPosition position;
    private PDR pdr;
    private FloorMap floor;

    public PDRStrategy(PDR pdr, XYPosition initialPosition, FloorMap floor) {
        this.position = initialPosition;
        this.pdr = pdr;
        this.floor = floor;
    }

    @Override
    public IndoorPosition getCurrentPosition() {
        return new IndoorPosition(position,floor.getFloor(),System.currentTimeMillis());
    }

    @Override
    protected void startEmission() {
        pdr.register(this);
    }

    @Override
    protected void stopEmission() {
        pdr.unregister(this);
    }

    @Override
    public void notify(PDR.Result data) {
        float x = position.x + data.dE;
        float y = position.y + data.dN;
        XYPosition p;
        if(floor != null)
            p = floor.nearestValid(x,y);
        else
            p = new XYPosition(x,y);
        notifyObservers(new IndoorPosition(p,floor.getFloor(),System.currentTimeMillis()));
    }
}
