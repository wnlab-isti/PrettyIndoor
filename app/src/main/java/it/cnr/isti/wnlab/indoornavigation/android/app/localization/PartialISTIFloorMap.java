package it.cnr.isti.wnlab.indoornavigation.android.app.localization;

import android.util.Log;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.map.RoomMap;

/**
 * A rough class representing a portion of the ISTI floor (the one in the sheet).
 */
class PartialISTIFloorMap extends FloorMap {

    private static RoomMap westernCorridor = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            return(x >= 0.f && y >= 4.8f // Bottom left
                    && x < 16.8f && y < 6.6f); // Up right
        }
    };

    private static RoomMap westernDoors = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            return (x >= 16.8f && y >= 4.8f // Bottom left
                    && x < 19.8f && y < 6.6f); // Up right
        }
    };

    private static RoomMap vendingMachines = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            return (x >= 19.2f && y >= 4.2f // Bottom left
                    && x < 24.6f && y < 8.4f); // Up right
        }
    };

    private static RoomMap easternHorizontalCorridor = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            return (x >= 24.0f && y >= 4.8f // Bottom left
                    && x < 34.8f && y < 6.6f); // Up right
        }
    };

    private static RoomMap easternVerticalCorridor = new RoomMap() {
        @Override
        public boolean isValid(float x, float y) {
            return (x >= 34.8f && y >= 4.8f // Bottom left
                    && x < 36.6f && y < 29.4f); // Up right
        }
    };

    PartialISTIFloorMap() {
        super(
                1,
                westernCorridor,
                westernDoors,
                vendingMachines,
                easternHorizontalCorridor,
                easternVerticalCorridor
        );
    }

    @Override
    public boolean isValid(float x, float y) {
        if(super.isValid(x,y)) {
            Log.d("PFS", "Valid position");
            return true;
        } else {
            Log.d("PFS", "Invalid position");
            return false;
        }
    }

    @Override
    public XYPosition nearestValid(float x, float y) {
        Float newX = null;
        Float newY = null;

        // Brace yourself, else-ifs are coming
        if(x >= 36.6f && y >= 29.4f) { // North East
            newX = 36.6f; newY = 29.4f;

        } else if(x >= 34.8f && x <= 36.6f && y > 29.4f) { // North of eastern corridor
            newY = 29.4f;

        } else if(y >= 4.8f && y <= 29.4f && x > 36.6f) { // East of eastern corridor
            newX = 36.6f;

        } else if(x >= 36.6f && y <= 4.8f) { // South East
            newX = 36.6f; newY = 4.8f;

        } else if(x >= 0.f && x <= 36.6f && y < 4.8f) { // South
            newY = 4.8f;

        } else if(x <= 0.f && y <= 4.8f) { // South West
            newX = 0.f; newY = 4.8f;

        } else if(x < 0.f && y >= 4.8f && y <= 6.6f) { // West
            newX = 0.f;

        } else if(x <= 0.f && y >= 6.6f) { // North West
            newX = 0.f; newY = 6.6f;

        } else if(x <= 19.8f && y >= 8.4f) { // North West of vending machine room
            newX = 19.8f; newY = 8.4f;

        } else if(x <= 19.8f && y >= 6.6f) { // In rectangle (N: 8.4, E: 19.8, S: 6.6, W: 0)

            // Project y
            //float x1 = x;
            float y1 = 6.6f;

            // Project x
            float x2 = 19.8f;
            //float y2 = y;

            // Find min distance
            float dyProj = y1-y;
            float dxProj = x-x2;
            if(dyProj < dxProj) {
                newY = 6.6f;
            } else if(dyProj > dxProj) {
                newX = 19.8f;
            } else {
                newX = 19.8f;
                newY = 6.6f;
            }

        } else if(x >= 19.2f && x <= 24.6f && y > 8.4f) { // North of vending machines
            newY = 8.4f;

        } else if(x >= 24.f && x <= 29.4f && y > 6.6f) { // Between vending machines and eastern corridor

            // Project y
            float dy = y-6.6f;

            // Project x (left)
            float dxL = x-24.f;

            // Project x (right)
            float dxR = 34.8f-x;

            // Find min distance
            if(dy < dxL && dy < dxR)
                newY = 6.6f;
            else if(dxL < dy && dxL < dxR)
                newX = 24.f;
            else if(dxR < dy && dxR < dxL)
                newX = 34.8f;

        }

        // Return new computed position
        return new XYPosition(
                newX != null ? newX : x,
                newY != null ? newY : y
        );
    }
}
