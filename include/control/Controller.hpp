#pragma once

#include <concepts>
#include <tuple>

template <typename T>
concept ControllerType = requires(T controller, float setpoint, float measurement) {
    requires std::copy_constructible<T>;
    { controller.update(setpoint, measurement) } -> std::same_as<float>;
};

template <ControllerType... ControllerTypes>
class Controller {
public:
    Controller(ControllerTypes... controllers) : m_controllers{ controllers... } {}

    float update(float setpoint, float measurement) {
        return (std::get<ControllerTypes>(m_controllers).update(setpoint, measurement) + ...);
    }

private:
    std::tuple<ControllerTypes...> m_controllers;
};
