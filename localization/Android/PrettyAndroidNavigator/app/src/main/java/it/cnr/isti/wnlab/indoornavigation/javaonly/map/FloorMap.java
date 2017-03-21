package it.cnr.isti.wnlab.indoornavigation.javaonly.map;

import java.util.ArrayList;
import java.util.Collection;

public class FloorMap implements XYMap {

    private Collection<RoomMap> rooms;

    public FloorMap() {
        rooms = new ArrayList<>();
    }

    @Override
    public boolean isValid(float x, float y) {
        for(RoomMap room : rooms) {
            if(room.isValid(x,y))
                return true;
        }
        return false;
    }
}
