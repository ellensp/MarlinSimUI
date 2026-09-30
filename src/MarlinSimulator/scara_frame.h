#pragma once

#include <src/inc/MarlinConfig.h>

#if ENABLED(SCARA)

//
// Where a SCARA machine is in the simulator's world, and how Marlin's coordinates map onto it.
//
// The world is the one the other models use: the bed plate spans 0..X_BED_SIZE and 0..Y_BED_SIZE.
// The shoulder is part of the hardware, so it is fixed in the world: centered behind the bed. Marlin's
// coordinates are only a way of naming places in it. SCARA_OFFSET says where the shoulder is in them, so
// it fixes how they line up with the world, and moves nothing physical:
//
//     world = native - SCARA_OFFSET + shoulder
//
// A config whose SCARA_OFFSET, limits or bed don't match the hardware doesn't move the arm: the markings
// (reach, limits, probe area, mesh) and the places Marlin sends the tool land off the plate, as they would
// on a real machine set up that way.
//
namespace scara_frame {
  constexpr float shoulder_gap = 40;   // From the back edge of the bed to the shoulder axis

  constexpr float shoulder_x = float(X_BED_SIZE) * 0.5f, shoulder_y = float(Y_BED_SIZE) + shoulder_gap;

  // Add to a Marlin (native) XY to get the world XY
  constexpr float shift_x = shoulder_x - float(SCARA_OFFSET_X), shift_y = shoulder_y - float(SCARA_OFFSET_Y);
}

#endif // SCARA
