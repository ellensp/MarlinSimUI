#pragma once

#include <algorithm>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <functional>

#include <glm/glm.hpp>

#include <vector>

struct kinematic_state;

namespace agent { class JsonWriter; class JsonValue; }

class VirtualPrinter {
public:
  struct Component {
    Component() : identifier("UnnamedComponent") {}
    Component(std::string identifier) : identifier(identifier) {}
    virtual ~Component() {}

    virtual void update() {};
    virtual void ui_init() {};
    virtual void ui_widget() {};
    virtual void ui_widgets();
    bool ui_default_open() const;
    void request_ui_collapse();

    /**
     * Emit this component's state as a JSON object for the agent interface.
     *
     * Sibling of ui_widget(): where ui_widget() renders state for a human,
     * serialize() reports it to a machine. The default writes an empty object,
     * so components adopt this incrementally rather than in one flag day.
     *
     * Called on the simulation thread (see AgentServer). Keep it allocation
     * -light and never block.
     */
    virtual void serialize(agent::JsonWriter& writer) const;

    /**
     * Capture this component's screen for the agent interface.
     *
     * Components that render a display fill rgb with 3 bytes per pixel,
     * row-major, top row first, and set width/height. The default returns
     * false, meaning "not a display" -- so like serialize(), displays adopt
     * this one at a time.
     *
     * Called on the simulation thread. Reads the device's own CPU-side pixel
     * buffer, NOT the GL texture, so it needs no GL context and works
     * regardless of whether the window is visible or the UI pane is open.
     */
    virtual bool capture(std::vector<uint8_t>& rgb, uint32_t& width, uint32_t& height) const {
      (void)rgb; (void)width; (void)height;
      return false;
    }

    /**
     * Inject a touch for the agent interface, as if the screen were pressed at
     * (rx, ry) -- ratios 0..1 of the panel -- and released after hold_ms of
     * simulated time (0 = a tap as short as a mouse click). The default
     * returns false, meaning "not a touch device".
     *
     * Called on the simulation thread.
     */
    virtual bool inject_touch(float rx, float ry, uint32_t hold_ms) {
      (void)rx; (void)ry; (void)hold_ms;
      return false;
    }

    /**
     * Change this component's simulated state for the agent interface -- the
     * write-side sibling of serialize(). The body is the flat JSON object from
     * POST /component/<name>, with keys named after the serialize() fields, so
     * an agent sets e.g. {"filament_present": false} without knowing any pin.
     *
     * Return true if at least one key was applied. On a bad value set error and
     * return false. The default knows no keys, so it reports that.
     *
     * Called on the simulation thread.
     */
    virtual bool apply(const agent::JsonValue& body, std::string& error) {
      (void)body;
      error = "component '" + name + "' has no settable state";
      return false;
    }

    template<typename T, class... Args>
    auto add_component(std::string name, Args&&... args) {
      auto component = VirtualPrinter::add_component<T>(name, args...);
      component->parent = get_component<Component>(this->name);
      children.push_back(component);
      return component;
    }

    template<typename T>
    std::shared_ptr<T> get_child(std::string name) {
      auto child = std::find_if(std::begin(children), std::end(children), [name](auto val){ return val->name == name; });
      if (child != children.end()) return std::static_pointer_cast<T>(*child);
      return nullptr;
    }

    std::string name;
    const std::string identifier;

    std::shared_ptr<Component> parent;
    std::vector<std::shared_ptr<Component>> children;

    bool ui_initial_open_applied = false;
    bool ui_collapse_requested = false;
  };

  static void update() {
    for(auto const& it : components) it->update();
  }

  static void ui_widgets();
  static void collapse_components_list();

  // Serialize every registered component as {"<name>": {...}} for the agent
  // interface. Components that have not implemented serialize() report {}.
  static void serialize_all(agent::JsonWriter& writer);

  // Serialize a single component by registry name. Returns false if unknown.
  static bool serialize_one(const std::string& name, agent::JsonWriter& writer);

  // Names of all registered components, in registration order.
  static std::vector<std::string> component_names();

  // Capture a display component by registry name. When name is empty the first
  // component that reports a screen is used, which is what an agent wants on a
  // machine with a single display. Returns false if no match.
  static bool capture_display(const std::string& name,
                              std::vector<uint8_t>& rgb,
                              uint32_t& width, uint32_t& height,
                              std::string& matched_name);

  // Registry names of components that report a capturable screen.
  static std::vector<std::string> display_names();

  // Inject a touch into the first touch device. Returns false if there is none.
  static bool inject_touch(float rx, float ry, uint32_t hold_ms, std::string& matched_name);

  static void build();
  static void update_kinematics();

  template<typename T, class... Args>
  static auto add_component(std::string name, Args&&... args) {
    auto component = std::make_shared<T>(args...);
    component->name = name;
    components.push_back(component);
    component_map[name] = component;
    return component;
  }

  template<typename T>
  static std::shared_ptr<T> get_component(std::string name) {
    return std::static_pointer_cast<T>(component_map[name]);
  }

  // Look up a component by registry name without creating an empty entry.
  static std::shared_ptr<Component> find_component(const std::string& name) {
    auto it = component_map.find(name);
    return it == component_map.end() ? nullptr : it->second;
  }

  // Registered components with the given identifier (the class tag, e.g. "Heater").
  static std::vector<std::shared_ptr<Component>> components_of(const std::string& identifier) {
    std::vector<std::shared_ptr<Component>> found;
    for (auto const& c : components) if (c->identifier == identifier) found.push_back(c);
    return found;
  }

  static std::function<void(kinematic_state&)> on_kinematic_update;

private:
  static std::map<std::string, std::shared_ptr<Component>> component_map;
  static std::vector<std::shared_ptr<Component>> components;
  static std::shared_ptr<Component> root;
};
