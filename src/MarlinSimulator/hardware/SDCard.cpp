
#include "SDCard.h"
#include <src/sd/SdInfo.h>
#include <src/sd/SdFatConfig.h>
#include <imgui_internal.h>
#include <array>
#include <cstring>
#include <filesystem>
#include <system_error>
#ifndef _WIN32
  #include <sys/types.h>
#endif

namespace {
  std::string saved_sd_image_filename = SD_SIMULATOR_FAT_IMAGE;

  uint16_t read_le16(const uint8_t* p) {
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
  }

  uint32_t read_le32(const uint8_t* p) {
    return uint32_t(p[0])
         | (uint32_t(p[1]) << 8)
         | (uint32_t(p[2]) << 16)
         | (uint32_t(p[3]) << 24);
  }

  bool read_image_sector(FILE* fp, const uint32_t sector, uint8_t* dst) {
    constexpr uint64_t sector_size = 512;
    const uint64_t offset = uint64_t(sector) * sector_size;
#ifdef _WIN32
    if (_fseeki64(fp, static_cast<__int64>(offset), SEEK_SET) != 0) return false;
#else
    if (fseeko(fp, static_cast<off_t>(offset), SEEK_SET) != 0) return false;
#endif
    return fread(dst, sector_size, 1, fp) == 1;
  }

  bool validate_fat_volume(FILE* fp, const uint64_t image_sectors, const uint32_t volume_start) {
    std::array<uint8_t, 512> boot {};
    if (!read_image_sector(fp, volume_start, boot.data())) return false;
    if (boot[510] != 0x55 || boot[511] != 0xAA) return false;

    const uint16_t bytes_per_sector = read_le16(&boot[11]);
    const uint8_t sectors_per_cluster = boot[13];
    const uint16_t reserved_sector_count = read_le16(&boot[14]);
    const uint8_t fat_count = boot[16];
    const uint16_t root_dir_entries = read_le16(&boot[17]);
    const uint16_t total_sectors_16 = read_le16(&boot[19]);
    const uint16_t sectors_per_fat_16 = read_le16(&boot[22]);
    const uint32_t total_sectors_32 = read_le32(&boot[32]);
    const uint32_t sectors_per_fat_32 = read_le32(&boot[36]);

    if (bytes_per_sector != 512
        || fat_count == 0
        || reserved_sector_count == 0
        || sectors_per_cluster == 0
        || sectors_per_cluster > 128
        || (sectors_per_cluster & (sectors_per_cluster - 1)) != 0)
      return false;

    const uint32_t blocks_per_fat = sectors_per_fat_16 ? sectors_per_fat_16 : sectors_per_fat_32;
    const uint32_t total_blocks = total_sectors_16 ? total_sectors_16 : total_sectors_32;
    if (blocks_per_fat == 0 || total_blocks == 0) return false;

    const uint64_t fat_start = uint64_t(volume_start) + reserved_sector_count;
    const uint64_t root_dir_start = fat_start + uint64_t(fat_count) * blocks_per_fat;
    const uint64_t root_dir_blocks = (uint64_t(32) * root_dir_entries + 511) / 512;
    const uint64_t data_start = root_dir_start + root_dir_blocks;

    if (data_start <= volume_start) return false;
    const uint64_t non_data_blocks = data_start - volume_start;
    if (non_data_blocks >= total_blocks) return false;

    const uint64_t volume_end = uint64_t(volume_start) + total_blocks;
    if (volume_end > image_sectors) return false;

    const uint64_t cluster_count = (uint64_t(total_blocks) - non_data_blocks) / sectors_per_cluster;
    if (cluster_count < 4085) {
#if !FAT12_SUPPORT
      return false;
#endif
    }

    std::array<uint8_t, 512> probe {};
    if (fat_start >= volume_end || !read_image_sector(fp, static_cast<uint32_t>(fat_start), probe.data()))
      return false;

    uint64_t root_sector;
    if (cluster_count < 65525) {
      if (root_dir_entries == 0) return false;
      root_sector = root_dir_start;
    }
    else {
      const uint32_t root_cluster = read_le32(&boot[44]);
      if (root_cluster < 2 || uint64_t(root_cluster) > cluster_count + 1) return false;
      root_sector = data_start + uint64_t(root_cluster - 2) * sectors_per_cluster;
    }

    return root_sector < volume_end
        && read_image_sector(fp, static_cast<uint32_t>(root_sector), probe.data());
  }

  bool validate_fat_image(FILE* fp, const uint64_t image_sectors) {
    std::array<uint8_t, 512> sector_zero {};
    if (!read_image_sector(fp, 0, sector_zero.data())) return false;

    constexpr size_t partition_offset = 446;
    const uint8_t partition_boot = sector_zero[partition_offset];
    const uint32_t first_sector = read_le32(&sector_zero[partition_offset + 8]);
    const uint32_t partition_sectors = read_le32(&sector_zero[partition_offset + 12]);

    const bool valid_partition = (partition_boot & 0x7F) == 0
                              && partition_sectors >= 100
                              && first_sector != 0
                              && uint64_t(first_sector) + partition_sectors <= image_sectors;

    if (valid_partition && validate_fat_volume(fp, image_sectors, first_sector))
      return true;

    return validate_fat_volume(fp, image_sectors, 0);
  }
}

constexpr char empty_disk_100[] =
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFE"
    "\xFF\xFF\x0C\xFE\xFF\xFF\x01\x00\x00\x00\x2F\xFB\x02\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x55\xAA"
    "\xEB\x3C\x90\x6D\x6B\x66\x73\x2E\x66\x61\x74\x00\x02\x04\x04\x00"
    "\x02\x00\x02\x00\x00\xF8\xC0\x00\x20\x00\x40\x00\x00\x00\x00\x00"
    "\x2F\xFB\x02\x00\x80\x00\x29\x44\x48\x36\xEA\x53\x44\x43\x41\x52"
    "\x44\x49\x4D\x41\x47\x45\x46\x41\x54\x31\x36\x20\x20\x20\x0E\x1F"
    "\xBE\x5B\x7C\xAC\x22\xC0\x74\x0B\x56\xB4\x0E\xBB\x07\x00\xCD\x10"
    "\x5E\xEB\xF0\x32\xE4\xCD\x16\xCD\x19\xEB\xFE\x54\x68\x69\x73\x20"
    "\x69\x73\x20\x6E\x6F\x74\x20\x61\x20\x62\x6F\x6F\x74\x61\x62\x6C"
    "\x65\x20\x64\x69\x73\x6B\x2E\x20\x20\x50\x6C\x65\x61\x73\x65\x20"
    "\x69\x6E\x73\x65\x72\x74\x20\x61\x20\x62\x6F\x6F\x74\x61\x62\x6C"
    "\x65\x20\x66\x6C\x6F\x70\x70\x79\x20\x61\x6E\x64\x0D\x0A\x70\x72"
    "\x65\x73\x73\x20\x61\x6E\x79\x20\x6B\x65\x79\x20\x74\x6F\x20\x74"
    "\x72\x79\x20\x61\x67\x61\x69\x6E\x20\x2E\x2E\x2E\x20\x0D\x0A\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x55\xAA";
// + zeroed 100031488 bytes

void SDCard::register_imgui_settings() {
  ImGuiSettingsHandler handler;
  handler.TypeName = "MarlinSimUI";
  handler.TypeHash = ImHashStr(handler.TypeName);
  handler.UserData = &saved_sd_image_filename;

  handler.ReadOpenFn = [](ImGuiContext*, ImGuiSettingsHandler* handler, const char* name) -> void* {
    return std::strcmp(name, "SDCard") == 0 ? handler->UserData : nullptr;
  };

  handler.ReadLineFn = [](ImGuiContext*, ImGuiSettingsHandler*, void* entry, const char* line) {
    if (std::strncmp(line, "Image=", 6) == 0)
      *static_cast<std::string*>(entry) = line + 6;
  };

  handler.WriteAllFn = [](ImGuiContext*, ImGuiSettingsHandler* handler, ImGuiTextBuffer* out_buf) {
    const auto* filename = static_cast<const std::string*>(handler->UserData);
    out_buf->appendf("[%s][SDCard]\n", handler->TypeName);
    out_buf->appendf("Image=%s\n\n", filename->c_str());
  };

  ImGui::AddSettingsHandler(&handler);
}

const std::string& SDCard::get_saved_image_filename() {
  return saved_sd_image_filename;
}

void SDCard::set_saved_image_filename(const std::string& filename) {
  saved_sd_image_filename = filename;
  ImGui::MarkIniSettingsDirty();
}

SDCard::ImgAuditStatus SDCard::audit_image() const {
  if (image_filename.empty()) return ImgAuditStatus::Missing;

  std::error_code ec;
  const auto path = std::filesystem::u8path(image_filename);
  if (!std::filesystem::exists(path, ec) || ec) return ImgAuditStatus::Missing;
  if (!std::filesystem::is_regular_file(path, ec) || ec) return ImgAuditStatus::Corrupted;

  const auto image_size = std::filesystem::file_size(path, ec);
  if (ec || image_size < 512 || (image_size % 512) != 0) return ImgAuditStatus::Corrupted;

  auto image_fp = fopen_utf8(image_filename, "rb+");
  if (!image_fp) return ImgAuditStatus::Corrupted;

  const bool valid = validate_fat_image(image_fp, image_size / 512);
  fclose(image_fp);
  return valid ? ImgAuditStatus::Passed : ImgAuditStatus::Corrupted;
}

void SDCard::refresh_img_audit() {
  img_audit_status = audit_image();
}

void SDCard::draw_img_audit() const {
  const bool passed = img_audit_status == ImgAuditStatus::Passed;
  const char* status_text =
    passed ? "Passed" :
    img_audit_status == ImgAuditStatus::Missing ? "File is missing!" : "File is corrupted!";

  ImGui::TextUnformatted("[IMG Audit:");
  ImGui::SameLine(0.0f, 4.0f);

  constexpr float square_size = 8.0f;
  const float line_height = ImGui::GetTextLineHeight();
  const ImVec2 square_pos = ImGui::GetCursorScreenPos();
  const float square_offset_y = (line_height - square_size) * 0.5f + 4.0f;
  const ImU32 square_color = ImGui::GetColorU32(
    passed ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f) : ImVec4(1.0f, 0.0f, 0.0f, 1.0f)
  );

  ImGui::GetWindowDrawList()->AddRectFilled(
    ImVec2(square_pos.x, square_pos.y + square_offset_y),
    ImVec2(square_pos.x + square_size, square_pos.y + square_offset_y + square_size),
    square_color
  );
  ImGui::Dummy(ImVec2(square_size, line_height));

  ImGui::SameLine(0.0f, 4.0f);
  ImGui::Text("%s]", status_text);
}

std::string SDCard::image_dialog_path(const std::string& filename) {
  if (filename.empty()) return ".";

  std::error_code ec;
  const auto parent = std::filesystem::u8path(filename).parent_path();
  if (parent.empty()
      || !std::filesystem::exists(parent, ec)
      || ec
      || !std::filesystem::is_directory(parent, ec)
      || ec)
    return ".";

  return parent.u8string();
}

FILE* SDCard::fopen_utf8(const std::string& filename, const char* mode) {
#ifdef _WIN32
  const auto wfilename = IGFD::Utils::UTF8Decode(filename);
  const auto wmode = IGFD::Utils::UTF8Decode(mode);
  return _wfopen(wfilename.c_str(), wmode.c_str());
#else
  return fopen(filename.c_str(), mode);
#endif
}

void SDCard::generate_empty_image(std::string filename) {
  auto fp = fopen_utf8(filename, "w+b");
  fwrite(empty_disk_100, sizeof(empty_disk_100) - 1, 1, fp);
  char zeros[512]{};
  for (std::size_t i = 0; i < 195374; i++) {
    fwrite(zeros, sizeof(zeros), 1, fp);
  }
  fclose(fp);
}

void SDCard::onByteReceived(uint8_t _byte) {
  SPISlavePeripheral::onByteReceived(_byte);
  if (getCurrentToken() != 0xFF || _byte == 0xFF) return;

  // 1 byte (cmd) + 4 byte (arg) + 1 byte (crc)
  const uint8_t cmd = _byte - 0x40;
  switch (cmd) {
    case CMD0:
    case CMD8:
    case CMD55:
    case CMD58:
    case CMD17: //read block
    case CMD24: //write block
    case CMD13:
    case ACMD41:
      setRequestedDataSize(cmd, 5);
      break;
  }
}

void SDCard::onRequestedDataReceived(uint8_t token, uint8_t* _data, size_t count) {
  SPISlavePeripheral::onRequestedDataReceived(token, _data, count);

  uint8_t crc = 0;
  // it should be handled per command, but no other call uses 5 bytes, so it's ok and simpler here
  if (count == 5) {
    currentArg = 0;
    for (int i = 0; i < 4; i++) {
      currentArg <<= 8;
      currentArg |= _data[i];
    }
    crc = _data[4];
  }

  UNUSED(crc);

  // Marlin SD2Card keep the CS LOW for multiple commands, so I need to manually clear the token, to receive next.
  clearCurrentToken();

  // printf("CMD: %d, currentArg: %d, crc: %d, count: %d\n", token, currentArg, crc, count);
  switch (token) {
    case CMD0:
      if (fp) fclose(fp);
      fp = fopen_utf8(image_filename, "rb+");
      if (fp)
        setResponse(R1_IDLE_STATE);
      else
        setResponse(0);
      break;
    case CMD8:
      if (true/*_type == SD_CARD_TYPE_SD1*/) {
        setResponse((R1_ILLEGAL_COMMAND | R1_IDLE_STATE));
      }
      else {
        memset(buf, 0xAA, 4);
        setResponse(buf, 4);
      }
      break;
    case CMD58:
      buf[0] = R1_READY_STATE;
      memset(buf+1, 0xC0, 3);
      setResponse(buf, 4);
      break;
    case CMD17: { //read block
      buf[0] = R1_READY_STATE;
      buf[1] = DATA_START_BLOCK;
      if (true  /*_type != SD_CARD_TYPE_SDHC*/) {
        currentArg >>= 9;
      }
      fseek(fp, 512 * currentArg, SEEK_SET);
      [[maybe_unused]] auto result = fread(buf + 2, 512, 1, fp);
      buf[512 + 2] = 0; //crc
      setResponse(buf, 512 + 3);
      break;
    }
    case CMD24: //write block
      if (true  /*_type != SD_CARD_TYPE_SDHC*/) {
        currentArg >>= 9;
      }
      setResponse(R1_READY_STATE);
      setRequestedDataSize(DATA_START_BLOCK, 512 + 1 + 2 + 1); //token + ff (from this response) + data + 2 crc
      break;
    case CMD13:
      setResponse16(R1_READY_STATE);
      break;
    case CMD55:
    case ACMD41:
      setResponse(R1_READY_STATE);
      break;
    case DATA_START_BLOCK: // CMD24 write block
      fseek(fp, 512 * currentArg, SEEK_SET);
      fwrite(_data + 2, 512, 1, fp);
      fflush(fp);
      buf[0] = DATA_RES_ACCEPTED;
      buf[1] = 0xFF; // ack for finish write
      setResponse(buf, 2);
      break;
  }
}
