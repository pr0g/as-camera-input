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
  const asc::Camera& target_camera, const as::real delta_time)
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

void Cameras::removeCamera(CameraInput* camera_input)
{
  if (auto camera_it = std::find(
        idle_camera_inputs_.begin(), idle_camera_inputs_.end(), camera_input);
      camera_it != idle_camera_inputs_.end()) {
    idle_camera_inputs_.erase(camera_it);
  }
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
  const int32_t scroll_delta, const as::real delta_time)
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

  std::sort(
    active_camera_inputs_.begin(), active_camera_inputs_.end(),
    [](const CameraInput* lhs, const CameraInput* rhs) {
      return lhs->priority() < rhs->priority();
    });

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

void handleEventsCommon(
  CameraInput& cameraInput, const InputEvent& event,
  const MouseButton button_type)
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
  using as::operator""_r;

  as::real x;
  as::real y;
  as::real z;

  // 2.3 Factor as RyRxRz
  if (orientation[as::mat3_rc(1, 2)] < 1.0_r) {
    if (orientation[as::mat3_rc(1, 2)] > -1.0_r) {
      x = std::asin(-orientation[as::mat3_rc(1, 2)]);
      y = std::atan2(
        orientation[as::mat3_rc(0, 2)], orientation[as::mat3_rc(2, 2)]);
      z = std::atan2(
        orientation[as::mat3_rc(1, 0)], orientation[as::mat3_rc(1, 1)]);
    } else {
      x = as::k_pi * 0.5_r;
      y = -std::atan2(
        -orientation[as::mat3_rc(0, 1)], orientation[as::mat3_rc(0, 0)]);
      z = 0.0_r;
    }
  } else {
    x = -as::k_pi * 0.5_r;
    y = std::atan2(
      -orientation[as::mat3_rc(0, 1)], orientation[as::mat3_rc(0, 0)]);
    z = 0.0_r;
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

  next_camera.pitch = wrapRotation(next_camera.pitch);
  next_camera.yaw = wrapRotation(next_camera.yaw);

  if (constrain_pitch_()) {
    // clamp pitch to be +-90 degrees
    next_camera.pitch =
      as::clamp(next_camera.pitch, -as::k_pi * 0.5_r, as::k_pi * 0.5_r);
  }

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

  translationDeltaFn_(next_camera, delta_pan_x * inv(props_.pan_invert_x_));
  translationDeltaFn_(next_camera, delta_pan_y * -inv(props_.pan_invert_y_));

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
      return TranslationType::Unset;
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
      if (translation_ != TranslationType::Unset) {
        beginActivation();
      }
      if (keyboard_button->button_ == KeyboardButton::LShift) {
        boost_ = true;
      }
    } else if (keyboard_button->action_ == ButtonAction::Up) {
      using bec::operator^=;
      translation_ ^= translationFromKey(keyboard_button->button_);
      if (translation_ == TranslationType::Unset) {
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
    translationDeltaFn_(next_camera, axis_z * speed * delta_time);
  }

  if ((translation_ & TranslationType::Backward) == TranslationType::Backward) {
    translationDeltaFn_(next_camera, -axis_z * speed * delta_time);
  }

  if ((translation_ & TranslationType::Left) == TranslationType::Left) {
    translationDeltaFn_(next_camera, -axis_x * speed * delta_time);
  }

  if ((translation_ & TranslationType::Right) == TranslationType::Right) {
    translationDeltaFn_(next_camera, axis_x * speed * delta_time);
  }

  if ((translation_ & TranslationType::Up) == TranslationType::Up) {
    translationDeltaFn_(next_camera, axis_y * speed * delta_time);
  }

  if ((translation_ & TranslationType::Down) == TranslationType::Down) {
    translationDeltaFn_(next_camera, -axis_y * speed * delta_time);
  }

  if (ending()) {
    translation_ = TranslationType::Unset;
  }

  return next_camera;
}

void TranslateCameraInput::resetImpl()
{
  translation_ = TranslationType::Unset;
  boost_ = false;
}

void PivotCameraInput::handleEvents(const InputEvent& event)
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
    pivot_cameras_.handleEvents(event);
  }
}

asc::Camera PivotCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  asc::Camera next_camera = target_camera;

  if (beginning()) {
    next_camera.pivot = pivotFn_();
    next_camera.offset =
      as::affine_transform_pos(next_camera.view(), target_camera.translation());
  }

  if (active()) {
    asc::move_pivot_detached(next_camera, pivotFn_());
    next_camera = pivot_cameras_.stepCamera(
      next_camera, cursor_delta, scroll_delta, delta_time);
  }

  if (ending()) {
    pivot_cameras_.reset();

    next_camera.pivot = next_camera.translation();
    next_camera.offset = as::vec3::zero();
  }

  return next_camera;
}

void PivotDollyScrollCameraInput::handleEvents(const InputEvent& event)
{
  if (const auto* scroll = std::get_if<ScrollEvent>(&event)) {
    beginActivation();
  }
}

static asc::Camera PivotDolly(
  const asc::Camera& target_camera, const as::real delta)
{
  using as::operator""_r;
  asc::Camera next_camera = target_camera;

  const auto pivot_direction = [&target_camera] {
    if (const auto offset_length = as::vec_length(target_camera.offset);
        as::real_near(offset_length, 0.0_r)) {
      return -as::vec3::axis_z();
    } else {
      return target_camera.offset / offset_length;
    }
  }();

  next_camera.offset -= pivot_direction * delta;

  if (as::vec_dot(pivot_direction, next_camera.offset) < 0.0_r) {
    next_camera.offset = pivot_direction * 0.001_r;
  }

  return next_camera;
}

asc::Camera PivotDollyScrollCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  const auto next_camera =
    PivotDolly(target_camera, as::real(scroll_delta) * props_.dolly_speed_);

  endActivation();

  return next_camera;
}

void PivotDollyMotionCameraInput::handleEvents(const InputEvent& event)
{
  handleEventsCommon(*this, event, button_type_);
}

asc::Camera PivotDollyMotionCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  return PivotDolly(
    target_camera, as::real(cursor_delta.y) * props_.dolly_speed_);
}

ScrollTranslationCameraInput::ScrollTranslationCameraInput()
{
  // default scroll axis is camera forward vector
  scroll_translation_axis_fn_ = [](const asc::Camera& next_camera) {
    return as::mat3_basis_z(lookTranslation(next_camera));
  };
}

void ScrollTranslationCameraInput::handleEvents(const InputEvent& event)
{
  if (const auto* scroll = std::get_if<ScrollEvent>(&event)) {
    beginActivation();
  }
}

asc::Camera ScrollTranslationCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  asc::Camera next_camera = target_camera;

  const auto translation_basis = lookTranslation(next_camera);
  const auto scroll_axis = scroll_translation_axis_fn_(next_camera);

  next_camera.pivot +=
    scroll_axis * as::real(scroll_delta) * props_.translate_speed_;

  endActivation();

  return next_camera;
}

void ScrollTranslationCameraInput::setScrollAxisFn(
  ScrollTranslationAxisFn scroll_translation_axis_fn)
{
  scroll_translation_axis_fn_ = std::move(scroll_translation_axis_fn);
}

asc::Camera smoothCamera(
  const asc::Camera& current_camera, const asc::Camera& target_camera,
  const SmoothProps& props, const as::real delta_time)
{
  const auto wrap_values = [](const as::real current, const as::real target) {
    as::real wrapped_target = wrapRotation(target);
    const as::real wrapped_current = wrapRotation(current);
    // ensure smooth transition when moving across 0-360 degree boundary
    if (const as::real delta = wrapped_target - wrapped_current;
        std::abs(delta) >= as::k_pi) {
      wrapped_target -= as::k_tau * as::sign(delta);
    }
    return std::pair(wrapped_current, wrapped_target);
  };

  const auto [current_roll, target_roll] =
    wrap_values(target_camera.roll, current_camera.roll);
  const auto [current_pitch, target_pitch] =
    wrap_values(target_camera.pitch, current_camera.pitch);
  const auto [current_yaw, target_yaw] =
    wrap_values(target_camera.yaw, current_camera.yaw);

  asc::Camera camera;
  // https://www.gamasutra.com/blogs/ScottLembcke/20180404/316046/Improved_Lerp_Smoothing.php
  const as::real look_rate = exp2(props.look_smoothness_);
  const as::real look_t = exp2(-look_rate * delta_time);
  camera.roll = as::mix(target_roll, current_roll, look_t);
  camera.pitch = as::mix(target_pitch, current_pitch, look_t);
  camera.yaw = as::mix(target_yaw, current_yaw, look_t);
  const as::real move_rate = exp2(props.move_smoothness_);
  const as::real move_t = exp2(-move_rate * delta_time);
  camera.offset = as::mix(target_camera.offset, current_camera.offset, move_t);
  camera.pivot = as::mix(target_camera.pivot, current_camera.pivot, move_t);
  return camera;
}

void FocusCameraInput::handleEvents(const InputEvent& event)
{
  if (
    const auto& keyboard_button =
      std::get_if<asci::KeyboardButtonEvent>(&event)) {
    if (
      keyboard_button->button_ == keyboard_button_
      && keyboard_button->action_ == asci::ButtonAction::Down) {
      beginActivation();
    }
  }
}

asc::Camera FocusCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  if (beginning()) {
    auto [forward, length] =
      as::vec_normalize_and_length(pivotFn_() - target_camera.translation());
    next_camera_.offset = offsetFn_(length);
    as::vec3 right, up;
    as::vec3_right_and_up_lh(forward, right, up);
    auto angles = asci::eulerAngles(as::mat3(right, up, forward));
    next_camera_.pitch = angles.x;
    next_camera_.yaw = angles.y;
    next_camera_.pivot = target_camera.pivot;
  }

  if (
    as::real_near(target_camera.pitch, next_camera_.pitch)
    && as::real_near(target_camera.yaw, next_camera_.yaw)) {
    endActivation();
  }

  return next_camera_;
}

void CustomCameraInput::handleEvents(const InputEvent& event)
{
  handleEventsFn_(*this, event);
}

asc::Camera CustomCameraInput::stepCamera(
  const asc::Camera& target_camera, const as::vec2i& cursor_delta,
  const int32_t scroll_delta, const as::real delta_time)
{
  return stepCameraFn_(
    *this, target_camera, cursor_delta, scroll_delta, delta_time);
}

} // namespace asci
