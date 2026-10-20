#include <OgreHardwarePixelBuffer.h>
#include <OgreTexture.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_default_plugins/displays/map/map_display.hpp>
#include <rviz_default_plugins/displays/map/palette_builder.hpp>

namespace occupancy_slam_rviz {
// Retain the stock Map implementation, including -1 unknown values and QoS.
// Only the palette entry for unknown is changed from teal to neutral gray.
class GrayscaleMap : public rviz_default_plugins::displays::MapDisplay {
public:
  void onInitialize() override {
    MapDisplay::onInitialize();
    updateUnknownColor();
    // The base slot rebuilds the binary palette first when its threshold changes.
    QObject::connect(binary_threshold_property_, &rviz_common::properties::Property::changed,
                     this, [this]() { updateUnknownColor(); });
  }
private:
  void updateUnknownColor() {
    for (bool binary : {false, true}) {
      auto palette = rviz_default_plugins::displays::makeMapPalette(
          binary, binary_threshold_property_->getInt());
      for (int channel = 0; channel < 3; ++channel) {
        palette[255 * 4 + channel] = 128;
      }
      auto & texture = (binary ? palette_textures_binary_ : palette_textures_).at(0);
      const Ogre::PixelBox pixels(256, 1, 1, Ogre::PF_BYTE_RGBA, palette.data());
      texture->getBuffer()->blitFromMemory(pixels);
    }
  }
};
}  // namespace occupancy_slam_rviz

PLUGINLIB_EXPORT_CLASS(occupancy_slam_rviz::GrayscaleMap, rviz_common::Display)
