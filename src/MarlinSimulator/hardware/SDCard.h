#pragma once

#include "../user_interface.h"

#include "SPISlavePeripheral.h"

/**
  * Instructions for create a FAT image:
  * 1) Install mtools
  * 2) Create the image file:
  *    $ mformat -v "EMBEDDED FS" -t 1 -h 1 -s 10000 -S 2 -C -i fs.img -c 1 -r 1 -L 1
  *    -s NUM is the number of sectors
  * 3) Copy files to the image:
  *    $ mcopy -i fs.img CFFFP_flow_calibrator.gcode ::/
  * 4) Set the path for SD_SIMULATOR_FAT_IMAGE
  */
 //#define SD_SIMULATOR_FAT_IMAGE "/full/path/to/fs.img"
#ifndef SD_SIMULATOR_FAT_IMAGE
  #define SD_SIMULATOR_FAT_IMAGE "fs.img"
#endif

class SDCard: public SPISlavePeripheral {
public:
  enum class ImgAuditStatus : uint8_t {
    Passed,
    Missing,
    Corrupted
  };

  static void register_imgui_settings();
  static const std::string& get_saved_image_filename();
  static void set_saved_image_filename(const std::string& filename);

  SDCard(SpiBus& spi_bus, pin_type cs, pin_type sd_detect = -1, bool sd_detect_state = true) : SPISlavePeripheral(spi_bus, cs), sd_detect(sd_detect), sd_detect_state(sd_detect_state), image_filename(get_saved_image_filename()) {
    if (Gpio::valid_pin(sd_detect)) {
      Gpio::attach(sd_detect, [this](GpioEvent& event){ this->interrupt(event); });
    }
    refresh_img_audit();
    sd_present = img_audit_status == ImgAuditStatus::Passed;
    Gpio::set_pin_value(sd_detect, sd_present);
  }
  virtual ~SDCard() {};

  void update() {}
  const std::string file_dialog_key = "ChooseSDFileDlgKey";
  void ui_widget() {
    if (ImGui::Button("Select Image (FAT32)")) {
      IGFD::FileDialogConfig config { image_dialog_path(image_filename) };
      config.flags |= ImGuiFileDialogFlags_Modal;
      ImGuiFileDialog::Instance()->OpenDialog(file_dialog_key, "Choose File", "FAT32 Disk Image(*.img){.img},.*", config);
    }
    ImVec4 file_dialog_bg = ImGui::GetStyleColorVec4(ImGuiCol_PopupBg);
    file_dialog_bg.w = 1.0f;
    ImGui::PushStyleColor(ImGuiCol_PopupBg, file_dialog_bg);

    if (ImGuiFileDialog::Instance()->IsOpened()) {
      const ImGuiViewport* viewport = ImGui::GetMainViewport();
      ImGui::SetNextWindowPos(
        viewport->GetCenter(),
        ImGuiCond_Appearing,
        ImVec2(0.5f, 0.5f)
      );
    }

    const bool file_dialog_done = ImGuiFileDialog::Instance()->Display(
      file_dialog_key,
      ImGuiWindowFlags_NoDocking,
      ImVec2(900.0f, 600.0f)
    );

    ImGui::PopStyleColor();

    if (file_dialog_done) {
      if (ImGuiFileDialog::Instance()->IsOk()) {
        image_filename = ImGuiFileDialog::Instance()->GetFilePathName();
        set_saved_image_filename(image_filename);
        refresh_img_audit();
        sd_present = img_audit_status == ImgAuditStatus::Passed;
        Gpio::set_pin_value(sd_detect, sd_present);
      }
      ImGuiFileDialog::Instance()->Close();
    }

    ImGui::Text("FileSystem image \"%s\" selected", image_filename.c_str());
    if (Gpio::valid_pin(sd_detect)) {
      bool requested_sd_present = sd_present;
      if (ImGui::Checkbox("SD Card Present ", &requested_sd_present)) {
        if (requested_sd_present) refresh_img_audit();
        sd_present = requested_sd_present && img_audit_status == ImgAuditStatus::Passed;
        Gpio::set_pin_value(sd_detect, sd_present);
      }
      ImGui::SameLine();
    }
    draw_img_audit();

    if (!sd_present) {
      if (ImGui::Button("Generate Empty Image")) {
        if (image_exists()) {
          printf("File exists unable to create new image!\n");
          //confirm overwrite?
        } else {
          generate_empty_image(image_filename);
          refresh_img_audit();
          sd_present = img_audit_status == ImgAuditStatus::Passed;
          Gpio::set_pin_value(sd_detect, sd_present);
        }
      }
    }
  }

  void onByteReceived(uint8_t _byte) override;
  void onRequestedDataReceived(uint8_t token, uint8_t* _data, size_t count) override;

  /**
   * Put a different image in the slot, as if the card were swapped. Marlin
   * reopens the image on its next mount (CMD0), so follow this with M21.
   * persist=false leaves the image a human picked in the UI remembered for
   * next launch. Returns false if the image fails the FAT audit.
   */
  bool use_image(const std::string& filename, const bool persist=false);

  // Insert or eject the card (the SD_DETECT switch). Inserting needs a valid image.
  bool set_present(const bool present);

  void serialize(agent::JsonWriter& writer) const override;
  // {"present": bool} ejects/inserts; {"image": "/abs/path.img"} swaps the image
  bool apply(const agent::JsonValue& body, std::string& error) override;

  void interrupt(GpioEvent &ev) {
    if (ev.pin_id == sd_detect && ev.event == GpioEvent::GET_VALUE) {
      Gpio::set_pin_value(sd_detect, sd_present ? sd_detect_state : !sd_detect_state);
    }
  }

  static FILE* fopen_utf8(const std::string& filename, const char* mode);
  static std::string image_dialog_path(const std::string& filename);

  ImgAuditStatus audit_image() const;
  void refresh_img_audit();
  void draw_img_audit() const;

  bool image_exists() {
    auto image_fp = fopen_utf8(image_filename, "rb+");
    if (image_fp == nullptr) {
      return false;
    }
    fclose(image_fp);
    return true;
  }
  void generate_empty_image(std::string filename);

  int32_t currentArg = 0;
  uint8_t buf[1024];
  FILE *fp = nullptr;
  ImgAuditStatus img_audit_status = ImgAuditStatus::Missing;
  bool sd_present = false;
  pin_type sd_detect;
  bool sd_detect_state = true;
  std::string image_filename;
};
