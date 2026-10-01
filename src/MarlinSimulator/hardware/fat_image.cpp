#include "fat_image.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <set>
#include <system_error>

namespace fs = std::filesystem;

namespace fat_image {

namespace {

//
// FAT16 layout used here, in 512-byte sectors:
//
//   [boot sector][FAT #1][FAT #2][root directory (512 entries)][data clusters ...]
//
// Cluster size is picked so the file data fits in 4085..65524 clusters,
// the range that makes a volume FAT16 (below it is FAT12, above it FAT32).
//
constexpr uint32_t sector_size = 512;
constexpr uint16_t root_entries = 512;
constexpr uint32_t root_sectors = root_entries * 32 / sector_size;
constexpr uint32_t min_clusters = 4200;    // Comfortably above the FAT12 limit of 4084
constexpr uint32_t max_clusters = 65524;   // FAT16 limit

void put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
void put32(uint8_t* p, uint32_t v) { put16(p, v & 0xFFFF); put16(p + 2, v >> 16); }

// Characters allowed in an 8.3 name besides A-Z and 0-9
bool short_name_char(const char c) {
  return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || strchr("!#$%&'()-@^_`{}~", c);
}

// Uppercase and drop characters not allowed in 8.3 names. Sets 'lossy' if anything changed.
std::string sanitize(const std::string& s, bool& lossy) {
  std::string out;
  for (char c : s) {
    const char u = (c >= 'a' && c <= 'z') ? char(c - 'a' + 'A') : c;
    if (short_name_char(u)) out += u;
    else if (c != ' ') { out += '_'; lossy = true; }
    else lossy = true;
  }
  return out;
}

// Assign an 8.3 name, unique among 'used': "NAME.EXT", or "NAME~N.EXT" when truncated
std::string make_short_name(const std::string& long_name, std::set<std::string>& used) {
  const size_t dot = long_name.find_last_of('.');
  const bool has_ext = dot != std::string::npos && dot > 0;
  bool lossy = false;
  std::string base = sanitize(has_ext ? long_name.substr(0, dot) : long_name, lossy),
              ext  = has_ext ? sanitize(long_name.substr(dot + 1), lossy) : "";
  if (base.empty()) { base = "FILE"; lossy = true; }
  if (ext.size() > 3) { ext.resize(3); lossy = true; }

  auto join = [&](const std::string& b) { return ext.empty() ? b : b + "." + ext; };

  if (!lossy && base.size() <= 8 && !used.count(join(base))) {
    used.insert(join(base));
    return join(base);
  }
  for (int n = 1; n < 1000000; ++n) {
    const std::string tail = "~" + std::to_string(n);
    const std::string candidate = join(base.substr(0, 8 - tail.size()) + tail);
    if (!used.count(candidate)) { used.insert(candidate); return candidate; }
  }
  return join(base.substr(0, 8));  // Unreachable in practice
}

// The 11-byte space-padded form of "NAME.EXT" stored in a directory entry
std::array<uint8_t, 11> dir_name(const std::string& short_name) {
  std::array<uint8_t, 11> n;
  n.fill(' ');
  const size_t dot = short_name.find('.');
  const std::string base = short_name.substr(0, dot),
                    ext = dot == std::string::npos ? "" : short_name.substr(dot + 1);
  memcpy(n.data(), base.data(), std::min<size_t>(8, base.size()));
  memcpy(n.data() + 8, ext.data(), std::min<size_t>(3, ext.size()));
  return n;
}

// VFAT checksum of the short name, stored in each of its long-name entries
uint8_t lfn_checksum(const std::array<uint8_t, 11>& n) {
  uint8_t sum = 0;
  for (uint8_t c : n) sum = uint8_t(((sum & 1) << 7) + (sum >> 1) + c);
  return sum;
}

// UTF-8 to UTF-16 (BMP only; anything else becomes '_')
std::u16string utf16(const std::string& s) {
  std::u16string out;
  for (size_t i = 0; i < s.size();) {
    const uint8_t c = s[i];
    uint32_t cp = '_'; size_t len = 1;
    if (c < 0x80) cp = c;
    else if ((c & 0xE0) == 0xC0 && i + 1 < s.size()) { cp = ((c & 0x1F) << 6) | (s[i + 1] & 0x3F); len = 2; }
    else if ((c & 0xF0) == 0xE0 && i + 2 < s.size()) { cp = ((c & 0x0F) << 12) | ((s[i + 1] & 0x3F) << 6) | (s[i + 2] & 0x3F); len = 3; }
    else if ((c & 0xF8) == 0xF0) len = 4;
    out += char16_t(cp);
    i += len;
  }
  return out;
}

// Directory entries for one file: its long-name entries (last part first), then the 8.3 entry
std::vector<std::array<uint8_t, 32>> dir_entries(const Entry& e, const uint16_t cluster, const uint32_t size) {
  std::vector<std::array<uint8_t, 32>> out;
  const auto sfn = dir_name(e.short_name);

  // Long-name entries hold 13 UTF-16 characters each at these byte offsets
  static constexpr uint8_t lfn_offsets[13] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };
  const std::u16string name = utf16(e.long_name);
  const int parts = int((name.size() + 12) / 13);
  for (int seq = parts; seq >= 1; --seq) {
    std::array<uint8_t, 32> d {};
    d[0] = uint8_t(seq | (seq == parts ? 0x40 : 0));  // 0x40 marks the last part
    d[11] = 0x0F;                                      // LFN attribute
    d[13] = lfn_checksum(sfn);
    for (int k = 0; k < 13; ++k) {
      const size_t idx = size_t(seq - 1) * 13 + k;
      const uint16_t ch = idx < name.size() ? uint16_t(name[idx]) : idx == name.size() ? 0x0000 : 0xFFFF;
      put16(&d[lfn_offsets[k]], ch);
    }
    out.push_back(d);
  }

  // The 8.3 entry, stamped with the current time
  const std::time_t now = std::time(nullptr);
  const std::tm* t = std::localtime(&now);
  const uint16_t dos_time = uint16_t((t->tm_hour << 11) | (t->tm_min << 5) | (t->tm_sec / 2)),
                 dos_date = uint16_t(((t->tm_year - 80) << 9) | ((t->tm_mon + 1) << 5) | t->tm_mday);
  std::array<uint8_t, 32> d {};
  memcpy(d.data(), sfn.data(), 11);
  d[11] = 0x20;  // Archive
  put16(&d[14], dos_time); put16(&d[16], dos_date); put16(&d[18], dos_date);
  put16(&d[22], dos_time); put16(&d[24], dos_date);
  put16(&d[26], cluster);
  put32(&d[28], size);
  out.push_back(d);
  return out;
}

} // namespace

std::vector<Entry> entries_in_folder(const std::string& dir) {
  std::vector<Entry> files;
  std::error_code ec;
  for (const auto& de : fs::directory_iterator(fs::u8path(dir), ec)) {
    if (!de.is_regular_file(ec)) continue;
    const std::string name = de.path().filename().u8string();
    if (name.empty() || name[0] == '.') continue;
    files.push_back({ de.path().u8string(), name, "" });
  }
  std::sort(files.begin(), files.end(), [](const Entry& a, const Entry& b){ return a.long_name < b.long_name; });
  return files;
}

bool build(const std::string& image_path, std::vector<Entry>& files, std::string& error) {
  // File sizes, and the root directory space their names need
  std::vector<uint32_t> sizes;
  uint64_t total_bytes = 0;
  size_t dir_slots = 0;
  std::set<std::string> used;
  for (auto& f : files) {
    std::error_code ec;
    const auto size = fs::file_size(fs::u8path(f.host_path), ec);
    if (ec) { error = "can't read '" + f.host_path + "'"; return false; }
    if (size > 0xFFFFFFFFull) { error = "'" + f.host_path + "' is over 4GB"; return false; }
    sizes.push_back(uint32_t(size));
    total_bytes += size;
    f.short_name = make_short_name(f.long_name, used);
    dir_slots += 1 + (utf16(f.long_name).size() + 12) / 13;
  }
  if (dir_slots > root_entries) { error = "too many files (or names too long) for the root directory"; return false; }

  // Smallest cluster size (in sectors) that keeps the cluster count within FAT16 limits
  uint32_t spc = 8, clusters = 0;
  for (;; spc <<= 1) {
    if (spc > 128) { error = "files are too large for a FAT16 image"; return false; }
    const uint32_t cluster_bytes = spc * sector_size;
    uint64_t needed = 0;
    for (auto s : sizes) needed += (uint64_t(s) + cluster_bytes - 1) / cluster_bytes;
    if (needed + 64 <= max_clusters) { clusters = std::max<uint32_t>(uint32_t(needed + 64), min_clusters); break; }
  }
  const uint32_t fat_sectors  = ((clusters + 2) * 2 + sector_size - 1) / sector_size,
                 data_start   = 1 + 2 * fat_sectors + root_sectors,
                 total        = data_start + clusters * spc;

  FILE* fp = fopen(image_path.c_str(), "w+b");
  if (!fp) { error = "can't create '" + image_path + "'"; return false; }
  auto write_at = [&](const uint64_t offset, const void* data, const size_t len) {
    return fseeko(fp, off_t(offset), SEEK_SET) == 0 && fwrite(data, 1, len, fp) == len;
  };
  bool ok = true;

  // Boot sector (BIOS parameter block)
  std::array<uint8_t, sector_size> boot {};
  const uint8_t jump[3] = { 0xEB, 0x3C, 0x90 };
  memcpy(&boot[0], jump, 3);
  memcpy(&boot[3], "MARLNSIM", 8);
  put16(&boot[11], sector_size);
  boot[13] = uint8_t(spc);
  put16(&boot[14], 1);                          // Reserved sectors (just this one)
  boot[16] = 2;                                 // FAT copies
  put16(&boot[17], root_entries);
  put16(&boot[19], total < 0x10000 ? uint16_t(total) : 0);
  boot[21] = 0xF8;                              // Media: fixed disk
  put16(&boot[22], uint16_t(fat_sectors));
  put16(&boot[24], 32); put16(&boot[26], 64);   // Nominal geometry
  put32(&boot[32], total < 0x10000 ? 0 : total);
  boot[36] = 0x80; boot[38] = 0x29;             // Drive number, extended boot signature
  put32(&boot[39], uint32_t(std::time(nullptr)));
  memcpy(&boot[43], "MARLIN SIM ", 11);
  memcpy(&boot[54], "FAT16   ", 8);
  boot[510] = 0x55; boot[511] = 0xAA;
  ok = ok && write_at(0, boot.data(), boot.size());

  // Lay out each file in consecutive clusters, building the FAT and root directory as we go
  std::vector<uint16_t> fat(clusters + 2, 0);
  fat[0] = 0xFFF8; fat[1] = 0xFFFF;
  std::vector<uint8_t> root(root_sectors * sector_size, 0);
  size_t slot = 0;
  uint32_t next_cluster = 2;
  const uint32_t cluster_bytes = spc * sector_size;
  std::vector<uint8_t> buffer(cluster_bytes);

  for (size_t i = 0; ok && i < files.size(); ++i) {
    const uint32_t count = (sizes[i] + cluster_bytes - 1) / cluster_bytes,
                   first = count ? next_cluster : 0;
    for (uint32_t c = 0; c < count; ++c)
      fat[first + c] = c + 1 < count ? uint16_t(first + c + 1) : 0xFFFF;

    // Copy the file's data, one cluster at a time
    FILE* in = fopen(files[i].host_path.c_str(), "rb");
    if (!in) { error = "can't open '" + files[i].host_path + "'"; ok = false; break; }
    for (uint32_t c = 0; ok && c < count; ++c) {
      const size_t got = fread(buffer.data(), 1, cluster_bytes, in);
      ok = write_at((uint64_t(data_start) + uint64_t(first + c - 2) * spc) * sector_size, buffer.data(), got);
    }
    fclose(in);

    for (const auto& d : dir_entries(files[i], uint16_t(first), sizes[i]))
      memcpy(&root[32 * slot++], d.data(), 32);
    next_cluster += count;
  }

  // Both FAT copies, the root directory, then extend the image to its full (sparse) size
  std::vector<uint8_t> fat_bytes(fat_sectors * sector_size, 0);
  for (size_t c = 0; c < fat.size(); ++c) put16(&fat_bytes[2 * c], fat[c]);
  ok = ok && write_at(uint64_t(1) * sector_size, fat_bytes.data(), fat_bytes.size())
          && write_at(uint64_t(1 + fat_sectors) * sector_size, fat_bytes.data(), fat_bytes.size())
          && write_at(uint64_t(1 + 2 * fat_sectors) * sector_size, root.data(), root.size());
  fclose(fp);

  if (ok) {
    std::error_code ec;
    fs::resize_file(fs::u8path(image_path), uint64_t(total) * sector_size, ec);
    if (ec) { error = "can't size '" + image_path + "'"; ok = false; }
  }
  else if (error.empty()) error = "write to '" + image_path + "' failed";
  return ok;
}

} // namespace fat_image
