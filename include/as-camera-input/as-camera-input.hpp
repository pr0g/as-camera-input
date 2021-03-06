#pragma once

#include "as-camera/as-camera.hpp"
#include "as/as-vec.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

namespace asci
{

using as::operator""_r;

struct CursorMotionEvent
{
  as::vec2i motion_;
};

struct ScrollEvent
{
  int32_t delta_;
};

enum class MouseButton
{
  Nil,
  Left,
  Right,
  Middle
};

enum class KeyboardButton
{
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z,
  LAlt,
  LShift,
  Ctrl,
  Nil
};

enum class ButtonAction
{
  Down,
  Up
};

struct MouseButtonEvent
{
  MouseButton button_;
  ButtonAction action_;
};

struct KeyboardButtonEvent
{
  KeyboardButton button_;
  ButtonAction action_;
  bool repeat_;
};

using InputEvent = std::variant<
  std::monostate, CursorMotionEvent, ScrollEvent, MouseButtonEvent,
  KeyboardButtonEvent>;

class CameraInput
{
public:
  enum class Activation
  {
    Idle,
    Begin,
    Active,
    End
  };

  virtual ~CameraInput() = default;

  bool beginning() const { return activation_ == Activation::Begin; }
  bool ending() const { return activation_ == Activation::End; }
  bool idle() const { return activation_ == Activation::Idle; }
  bool active() const { return activation_ == Activation::Active; }

  void beginActivation() { activation_ = Activation::Begin; }
  void endActivation() { activation_ = Activation::End; }
  void continueActivation() { activation_ = Activation::Active; }
  void clearActivation() { activation_ = Activation::Idle; }

  void reset()
  {
    clearActivation();
    resetImpl();
  }

  virtual void handleEvents(const InputEvent& event) = 0;
  virtual asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time) = 0;
  virtual bool exclusive() const { return false; }

protected:
  virtual void resetImpl() {}

private:
  Activation activation_ = Activation::Idle;
};

struct SmoothProps
{
  as::real look_smoothness_ = 5.0_r;
  as::real move_smoothness_ = 5.0_r;
};

asc::Camera smoothCamera(
  const asc::Camera& current_camera, const asc::Camera& target_camera,
  const SmoothProps& props, as::real delta_time);

class Cameras
{
public:
  void addCamera(CameraInput* camera_input);
  void handleEvents(const InputEvent& event);
  asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time);
  void reset();

private:
  std::vector<CameraInput*> active_camera_inputs_;
  std::vector<CameraInput*> idle_camera_inputs_;
};

class CameraSystem
{
public:
  void handleEvents(const InputEvent& event);
  asc::Camera stepCamera(const asc::Camera& target_camera, as::real delta_time);

  Cameras cameras_;

private:
  int32_t scroll_delta_ = 0;
  std::optional<as::vec2i> last_cursor_position_;
  std::optional<as::vec2i> current_cursor_position_;
};

class RotateCameraInput : public CameraInput
{
public:
  explicit RotateCameraInput(const MouseButton button_type)
    : button_type_(button_type)
  {
  }
  void handleEvents(const InputEvent& event) override;
  asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time) override;

  MouseButton button_type_;

  struct Props
  {
    as::real rotate_speed_ = 0.005_r;
  } props_;
};

struct PanAxes
{
  as::vec3 horizontal_axis_;
  as::vec3 vertical_axis_;
};

using PanAxesFn = std::function<PanAxes(const asc::Camera& camera)>;

inline PanAxes lookPan(const asc::Camera& camera)
{
  const as::mat3 orientation = camera.rotation();
  return {as::mat3_basis_x(orientation), as::mat3_basis_y(orientation)};
}

inline PanAxes orbitPan(const asc::Camera& camera)
{
  const as::mat3 orientation = camera.rotation();

  const auto basis_x = as::mat3_basis_x(orientation);
  const auto basis_z = [&orientation] {
    const auto forward = as::mat3_basis_z(orientation);
    return as::vec_normalize(as::vec3(forward.x, 0.0f, forward.z));
  }();

  return {basis_x, basis_z};
}

class PanCameraInput : public CameraInput
{
public:
  explicit PanCameraInput(PanAxesFn panAxesFn)
    : panAxesFn_(std::move(panAxesFn))
  {
  }
  void handleEvents(const InputEvent& event) override;
  asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time) override;

  struct Props
  {
    as::real pan_speed_ = 0.01_r;
    bool pan_invert_x_ = true;
    bool pan_invert_y_ = true;
  } props_;

private:
  PanAxesFn panAxesFn_;
};

using TranslationAxesFn = std::function<as::mat3(const asc::Camera& camera)>;

inline as::mat3 lookTranslation(const asc::Camera& camera)
{
  const as::mat3 orientation = camera.rotation();

  const auto basis_x = as::mat3_basis_x(orientation);
  const auto basis_y = as::vec3::axis_y();
  const auto basis_z = as::mat3_basis_z(orientation);

  return {basis_x, basis_y, basis_z};
}

inline as::mat3 orbitTranslation(const asc::Camera& camera)
{
  const as::mat3 orientation = camera.rotation();

  const auto basis_x = as::mat3_basis_x(orientation);
  const auto basis_y = as::vec3::axis_y();
  const auto basis_z = [&orientation] {
    const auto forward = as::mat3_basis_z(orientation);
    return as::vec_normalize(as::vec3(forward.x, 0.0f, forward.z));
  }();

  return {basis_x, basis_y, basis_z};
}

class TranslateCameraInput : public CameraInput
{
public:
  explicit TranslateCameraInput(TranslationAxesFn translationAxesFn)
    : translationAxesFn_(std::move(translationAxesFn))
  {
  }
  void handleEvents(const InputEvent& event) override;
  asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time) override;
  void resetImpl() override;

  struct Props
  {
    as::real translate_speed_ = 10.0_r;
    as::real boost_multiplier_ = 3.0_r;
  } props_;

private:
  enum class TranslationType
  {
    // clang-format off
    Nil     = 0,
    Forward  = 1 << 0,
    Backward = 1 << 1,
    Left     = 1 << 2,
    Right    = 1 << 3,
    Up       = 1 << 4,
    Down     = 1 << 5,
    // clang-format on
  };

  static TranslationType translationFromKey(KeyboardButton button);

  TranslationType translation_ = TranslationType::Nil;
  TranslationAxesFn translationAxesFn_;
  bool boost_ = false;
};

class OrbitDollyScrollCameraInput : public CameraInput
{
public:
  void handleEvents(const InputEvent& event) override;
  asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time) override;
  
  struct Props 
  {
    as::real dolly_speed_ = 0.2_r;
  } props_;
};

class OrbitDollyCursorMoveCameraInput : public CameraInput
{
public:
  void handleEvents(const InputEvent& event) override;
  asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time) override;
  
  struct Props 
  {
    as::real dolly_speed_ = 0.1_r;
  } props_;
};

class ScrollTranslationCameraInput : public CameraInput
{
public:
  void handleEvents(const InputEvent& event) override;
  asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time) override;

  struct Props 
  {
    as::real translate_speed_ = 0.2_r;
  } props_;
};

class OrbitCameraInput : public CameraInput
{
public:
  void handleEvents(const InputEvent& event) override;
  asc::Camera stepCamera(
    const asc::Camera& target_camera, const as::vec2i& cursor_delta,
    int32_t scroll_delta, as::real delta_time) override;
  bool exclusive() const override { return true; }

  Cameras orbit_cameras_;

  struct Props
  {
    as::real default_orbit_distance_ = 15.0_r;
    as::real max_orbit_distance_ = 100.0_r;
  } props_;
};

} // namespace asci
