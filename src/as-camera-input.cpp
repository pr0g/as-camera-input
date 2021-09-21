#include "as-camera-input/as-camera-input.hpp"

#include "bec/bitfield-enum-class.hpp"

template<>
struct bec::EnableBitMaskOperators<asci::TranslateCameraInput::TranslationType>
{
  static const bool Enable = true;
};

namespace asci
{

void CameraSystem::handleEvents(const InputEvent& event)
{
  if (const auto& cursor_motion = std::get_if<CursorMotionEvent>(&event)) {
    current_cursor_position_ = cursor_motion->motion_;
    // handle cursor warp gracefully
    if (
      current_cursor_position_.has_value()
      && last_cursor_position_.has_value()) {
      if (
        std::abs(current_cursor_position_->x - last_cursor_position_->x)
        >= 500) {
        last_cursor_position_->x = current_cursor_position_->x;
      }
      if (
        std::abs(current_cursor_position_->y - last_cursor_position_->y)
        >= 500) {
        last_cursor_position_->y = current_cursor_position_->y;
      }
    }
  } else if (const auto& scroll = std::get_if<ScrollEvent>(&event)) {
    scroll_delta_ = scroll->delta_;
  }

  cameras_.handleEvents(event);
}

asc::Camera CameraSystem::stepCamera(
  const asc::Camera& target_camera, as::real delta_time)
{
  const auto cursor_delta =
    current_cursor_position_.has_value() && last_cursor_position_.has_value()
      ? current_cursor_position_.value() - last_cursor_position_.value()
      : as::vec2i::zero();

  if (current_cursor_position_.has_value()) {
    last_cursor_position_ = current_cursor_position_;
  }

  const auto next_camera =
    cameras_.stepCamera(target_camera, cursor_delta, scroll_delta_, delta_time);

  scroll_delta_ = 0;

  return next_camera;
}

void Cameras::addCamera(CameraInput* camera_input)
{
  idle_camera_inputs_.push_back(camera_input);
}

void Cameras::handleEvents(const InputEvent& event)
{
  for (auto* camera_input : active_camera_inputs_) {
    camera_input->handleEvents(event);
  }

  for (auto* camera_input : idle_camera_inputs_) {
    camera_input->handleEvents(event);
  }
}

asc::Camera Cameras::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  int32_t scroll_delta, const as::real delta_time)
{
  for (int i = 0; i < idle_camera_inputs_.size();) {
    auto* camera_input = idle_camera_inputs_[i];
    const bool
      can_begin = camera_input->beginning()
               && std::all_of(
                    active_camera_inputs_.cbegin(),
                    active_camera_inputs_.cend(),
                    [](const auto& input) { return !input->exclusive(); })
               && (!camera_input->exclusive()
                   || (camera_input->exclusive() && active_camera_inputs_.empty()));
    if (can_begin) {
      active_camera_inputs_.push_back(camera_input);
      idle_camera_inputs_[i] =
        idle_camera_inputs_[idle_camera_inputs_.size() - 1];
      idle_camera_inputs_.pop_back();
    } else {
      i++;
    }
  }

  // accumulate
  asc::Camera next_camera = target_camera;
  for (auto* camera_input : active_camera_inputs_) {
    next_camera = camera_input->stepCamera(
      next_camera, cursor_delta, scroll_delta, delta_time);
  }

  for (int i = 0; i < active_camera_inputs_.size();) {
    auto* camera_input = active_camera_inputs_[i];
    if (camera_input->ending()) {
      camera_input->clearActivation();
      idle_camera_inputs_.push_back(camera_input);
      active_camera_inputs_[i] =
        active_camera_inputs_[active_camera_inputs_.size() - 1];
      active_camera_inputs_.pop_back();
    } else {
      camera_input->continueActivation();
      i++;
    }
  }

  return next_camera;
}

void Cameras::reset()
{
  for (int i = 0; i < active_camera_inputs_.size();) {
    active_camera_inputs_[i]->reset();
    idle_camera_inputs_.push_back(active_camera_inputs_[i]);
    active_camera_inputs_[i] =
      active_camera_inputs_[active_camera_inputs_.size() - 1];
    active_camera_inputs_.pop_back();
  }
}

static void handleEventsCommon(
  CameraInput& cameraInput, const InputEvent& event, MouseButton button_type)
{
  if (const auto& mouse_button = std::get_if<MouseButtonEvent>(&event)) {
    if (mouse_button->button_ == button_type) {
      if (mouse_button->action_ == ButtonAction::Down) {
        cameraInput.beginActivation();
      } else if (mouse_button->action_ == ButtonAction::Up) {
        cameraInput.endActivation();
      }
    }
  }
}

void RotateCameraInput::handleEvents(const InputEvent& event)
{
  handleEventsCommon(*this, event, button_type_);
}

// https://www.geometrictools.com/Documentation/EulerAngles.pdf
as::vec3 eulerAngles(const as::mat3& orientation)
{
  float x;
  float y;
  float z;

  // 2.4 Factor as RyRzRx
  if (orientation[as::mat3_rc(1, 0)] < 1.0f) {
    if (orientation[as::mat3_rc(1, 0)] > -1.0f) {
      x = std::atan2(
        -orientation[as::mat3_rc(1, 2)], orientation[as::mat3_rc(1, 1)]);
      y = std::atan2(
        -orientation[as::mat3_rc(2, 0)], orientation[as::mat3_rc(0, 0)]);
      z = std::asin(orientation[as::mat3_rc(1, 0)]);
    } else {
      x = 0.0f;
      y = -std::atan2(
        orientation[as::mat3_rc(2, 1)], orientation[as::mat3_rc(2, 2)]);
      z = -as::k_pi * 0.5f;
    }
  } else {
    x = 0.0f;
    y = std::atan2(
      orientation[as::mat3_rc(2, 1)], orientation[as::mat3_rc(2, 2)]);
    z = as::k_pi * 0.5f;
  }

  return as::vec3(x, y, z);
}

asc::Camera RotateCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  asc::Camera next_camera = target_camera;

  next_camera.pitch += as::real(cursor_delta[1]) * props_.rotate_speed_;
  next_camera.yaw += as::real(cursor_delta[0]) * props_.rotate_speed_;

  auto clamp_rotation = [](const as::real angle) {
    return std::fmod(angle + as::k_tau, as::k_tau);
  };

  next_camera.yaw = clamp_rotation(next_camera.yaw);
  // clamp pitch to be +-90 degrees
  next_camera.pitch =
    as::clamp(next_camera.pitch, -as::k_pi * 0.5_r, as::k_pi * 0.5_r);

  return next_camera;
}

void PanCameraInput::handleEvents(const InputEvent& event)
{
  handleEventsCommon(*this, event, button_type_);
}

asc::Camera PanCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  asc::Camera next_camera = target_camera;

  const auto pan_axes = panAxesFn_(next_camera);

  const auto delta_pan_x =
    as::real(cursor_delta.x) * pan_axes.horizontal_axis_ * props_.pan_speed_;
  const auto delta_pan_y =
    as::real(cursor_delta.y) * pan_axes.vertical_axis_ * props_.pan_speed_;

  const auto inv = [](const bool invert) {
    constexpr as::real Dir[] = {1.0_r, -1.0_r};
    return Dir[static_cast<int>(invert)];
  };

  next_camera.look_at += as::affine_inv_transform_dir(
    next_camera.transform(), delta_pan_x * inv(props_.pan_invert_x_));
  next_camera.look_at += as::affine_inv_transform_dir(
    next_camera.transform(), delta_pan_y * -inv(props_.pan_invert_y_));

  return next_camera;
}

TranslateCameraInput::TranslationType TranslateCameraInput::translationFromKey(
  KeyboardButton button)
{
  switch (button) {
    case KeyboardButton::W:
      return TranslationType::Forward;
    case KeyboardButton::S:
      return TranslationType::Backward;
    case KeyboardButton::A:
      return TranslationType::Left;
    case KeyboardButton::D:
      return TranslationType::Right;
    case KeyboardButton::Q:
      return TranslationType::Down;
    case KeyboardButton::E:
      return TranslationType::Up;
    default:
      return TranslationType::Nil;
  }
}

void TranslateCameraInput::handleEvents(const InputEvent& event)
{
  if (const auto& keyboard_button = std::get_if<KeyboardButtonEvent>(&event)) {
    if (keyboard_button->action_ == ButtonAction::Down) {
      if (keyboard_button->repeat_) {
        return;
      }
      using bec::operator|=;
      translation_ |= translationFromKey(keyboard_button->button_);
      if (translation_ != TranslationType::Nil) {
        beginActivation();
      }
      if (keyboard_button->button_ == KeyboardButton::LShift) {
        boost_ = true;
      }
    } else if (keyboard_button->action_ == ButtonAction::Up) {
      using bec::operator^=;
      translation_ ^= translationFromKey(keyboard_button->button_);
      if (translation_ == TranslationType::Nil) {
        endActivation();
      }
      if (keyboard_button->button_ == KeyboardButton::LShift) {
        boost_ = false;
      }
    }
  }
}

asc::Camera TranslateCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  asc::Camera next_camera = target_camera;

  const auto translation_basis = translationAxesFn_(next_camera);
  const auto axis_x = as::mat3_basis_x(translation_basis);
  const auto axis_y = as::mat3_basis_y(translation_basis);
  const auto axis_z = as::mat3_basis_z(translation_basis);

  using bec::operator&;

  const as::real speed = [boost = boost_, props = props_]() {
    return props.translate_speed_ * (boost ? props.boost_multiplier_ : 1.0_r);
  }();

  if ((translation_ & TranslationType::Forward) == TranslationType::Forward) {
    next_camera.look_at += as::affine_inv_transform_dir(
      next_camera.transform(), axis_z * speed * delta_time);
  }

  if ((translation_ & TranslationType::Backward) == TranslationType::Backward) {
    next_camera.look_at -= as::affine_inv_transform_dir(
      next_camera.transform(), axis_z * speed * delta_time);
  }

  if ((translation_ & TranslationType::Left) == TranslationType::Left) {
    next_camera.look_at -= as::affine_inv_transform_dir(
      next_camera.transform(), axis_x * speed * delta_time);
  }

  if ((translation_ & TranslationType::Right) == TranslationType::Right) {
    next_camera.look_at += as::affine_inv_transform_dir(
      next_camera.transform(), axis_x * speed * delta_time);
  }

  if ((translation_ & TranslationType::Up) == TranslationType::Up) {
    next_camera.look_at += as::affine_inv_transform_dir(
      next_camera.transform(), axis_y * speed * delta_time);
  }

  if ((translation_ & TranslationType::Down) == TranslationType::Down) {
    next_camera.look_at -= as::affine_inv_transform_dir(
      next_camera.transform(), axis_y * speed * delta_time);
  }

  if (ending()) {
    translation_ = TranslationType::Nil;
  }

  return next_camera;
}

void TranslateCameraInput::resetImpl()
{
  translation_ = TranslationType::Nil;
  boost_ = false;
}

void OrbitCameraInput::handleEvents(const InputEvent& event)
{
  if (const auto* keyboard_button = std::get_if<KeyboardButtonEvent>(&event)) {
    if (keyboard_button->button_ == KeyboardButton::LAlt) {
      if (!keyboard_button->repeat_) {
        if (keyboard_button->action_ == ButtonAction::Down) {
          beginActivation();
        } else if (keyboard_button->action_ == ButtonAction::Up) {
          endActivation();
        }
      }
    }
  }
  if (active()) {
    orbit_cameras_.handleEvents(event);
  }
}

static as::real intersectPlane(
  const as::vec3& origin, const as::vec3& direction, const as::vec4& plane)
{
  return -(as::vec_dot(origin, as::vec3_from_vec4(plane)) + plane.w)
       / as::vec_dot(direction, as::vec3_from_vec4(plane));
}

asc::Camera OrbitCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, as::real delta_time)
{
  asc::Camera next_camera = target_camera;

  if (beginning()) {
    as::real hit_distance = intersectPlane(
      target_camera.translation(), as::mat3_basis_z(target_camera.rotation()),
      as::vec4(as::vec3::axis_y()));

    if (hit_distance >= 0.0_r) {
      const as::real dist = std::min(hit_distance, props_.max_orbit_distance_);
      // next_camera.look_dist = -dist;
      next_camera.look_at = target_camera.translation()
                          + as::mat3_basis_z(target_camera.rotation()) * dist;
    } else {
      // next_camera.look_dist = -props_.default_orbit_distance_;
      next_camera.look_at = target_camera.translation()
                          + as::mat3_basis_z(target_camera.rotation())
                              * props_.default_orbit_distance_;
    }
  }

  if (active()) {
    // todo: need to return nested cameras to idle state when ending
    next_camera = orbit_cameras_.stepCamera(
      next_camera, cursor_delta, scroll_delta, delta_time);
  }

  if (ending()) {
    orbit_cameras_.reset();

    next_camera.look_at = next_camera.translation();
    // next_camera.look_dist = 0.0_r;
  }

  return next_camera;
}

void PivotDollyScrollCameraInput::handleEvents(const InputEvent& event)
{
  if (const auto* scroll = std::get_if<ScrollEvent>(&event)) {
    beginActivation();
  }
}

static asc::Camera Dolly(const asc::Camera& target_camera, const float delta)
{
  asc::Camera next_camera = target_camera;
  const auto pivot_direction =
    as::vec_normalize(target_camera.pivot - target_camera.translation());
  const auto pivot_transformed_direction = as::vec_normalize(
    as::affine_inv_transform_dir(target_camera.transform(), pivot_direction));
  next_camera.look_at =
    target_camera.look_at + pivot_transformed_direction * delta;
  auto pivot_dot = as::vec_dot(
    target_camera.translation() - target_camera.pivot,
    next_camera.translation() - target_camera.pivot);
  auto min_distance = 0.01f;
  auto distance =
    as::vec_distance(next_camera.pivot, next_camera.translation());
  if (distance < min_distance || pivot_dot < 0.0f) {
    next_camera.look_at = -pivot_transformed_direction * min_distance;
  }
  return next_camera;
}

asc::Camera PivotDollyScrollCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  int32_t scroll_delta, as::real delta_time)
{
  const auto next_camera =
    Dolly(target_camera, as::real(scroll_delta) * props_.dolly_speed_);
  endActivation();
  return next_camera;
}

void PivotDollyMotionCameraInput::handleEvents(const InputEvent& event)
{
  handleEventsCommon(*this, event, button_type_);
}

asc::Camera PivotDollyMotionCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, as::real delta_time)
{
  return Dolly(target_camera, as::real(cursor_delta.y) * props_.dolly_speed_);
}

void ScrollTranslationCameraInput::handleEvents(const InputEvent& event)
{
  if (const auto* scroll = std::get_if<ScrollEvent>(&event)) {
    beginActivation();
  }
}

asc::Camera ScrollTranslationCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  int32_t scroll_delta, as::real delta_time)
{
  asc::Camera next_camera = target_camera;

  const auto translation_basis = lookTranslation(next_camera);
  const auto axis_z = as::mat3_basis_z(translation_basis);

  next_camera.look_at +=
    axis_z * as::real(scroll_delta) * props_.translate_speed_;

  endActivation();

  return next_camera;
}

asc::Camera smoothCamera(
  const asc::Camera& current_camera, const asc::Camera& target_camera,
  const SmoothProps& props, const as::real delta_time)
{
  const auto clamp_rotation = [](const as::real angle) {
    return std::fmod(angle + as::k_tau, as::k_tau);
  };

  // keep yaw in 0 - 360 range
  as::real target_yaw = clamp_rotation(target_camera.yaw);
  const as::real current_yaw = clamp_rotation(current_camera.yaw);

  // ensure smooth transition when moving across 0 - 360 boundary
  const as::real yaw_delta = target_yaw - current_yaw;
  if (std::abs(yaw_delta) >= as::k_pi) {
    target_yaw -= as::k_tau * as::sign(yaw_delta);
  }

  asc::Camera camera;
  // https://www.gamasutra.com/blogs/ScottLembcke/20180404/316046/Improved_Lerp_Smoothing.php
  const as::real look_rate = exp2(props.look_smoothness_);
  const as::real look_t = exp2(-look_rate * delta_time);
  camera.pitch = as::mix(target_camera.pitch, current_camera.pitch, look_t);
  camera.yaw = as::mix(target_yaw, current_yaw, look_t);
  const as::real move_rate = exp2(props.move_smoothness_);
  const as::real move_t = exp2(-move_rate * delta_time);
  camera.look_at =
    as::mix(target_camera.look_at, current_camera.look_at, move_t);
  camera.pivot = as::mix(target_camera.pivot, current_camera.pivot, move_t);
  return camera;
}

} // namespace asci
