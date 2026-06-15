#pragma once

namespace drone {

// Values stored in the building map (drone's own map and output file)
enum class MapValue : int {
    Empty        =  0,  // position is known to be free
    Occupied     =  1,  // position is known to have an element (wall/floor/ceiling/obstacle)
    NotMapped    = -1,  // position was not reached / could not be mapped
    BeyondBounds = -2   // position is outside the required mapping boundaries
};

} // namespace drone
