#pragma once

/**
 * Build a FAT16 SD card image from host files, so an agent can print a G-code
 * file without preparing an image by hand (and without mtools or root).
 *
 * The image is a "superfloppy": FAT16 boot sector in block 0, no partition
 * table. Marlin's SdVolume::init() tries partition 1 first, finds no valid
 * entry, then falls back to block 0 -- the same path the simulator's own
 * image audit takes.
 *
 * Every file goes in the root directory with an 8.3 short name plus VFAT long
 * name entries, so Marlin lists and displays the original name. Subdirectories
 * are not recreated: a folder is packed flat, one level deep.
 */

#include <string>
#include <vector>

namespace fat_image {

struct Entry {
  std::string host_path;   // File to copy in
  std::string long_name;   // Name on the card (normally the host file name)
  std::string short_name;  // Assigned 8.3 name, "NAME.EXT" -- filled in by build()
};

// Write a FAT16 image containing 'files' to image_path, replacing it.
// Assigns each entry's short_name. Returns false and sets error on failure.
bool build(const std::string& image_path, std::vector<Entry>& files, std::string& error);

// The regular files directly inside dir, sorted by name, ready for build().
// Hidden files (leading '.') are skipped.
std::vector<Entry> entries_in_folder(const std::string& dir);

} // namespace fat_image
