#pragma once

using uint = unsigned int;

class Button {
public:
    Button(uint pin);

    bool isPressed() const;

    void waitForPress() const;
    void waitForUnpress() const;

    void waitForClick() const;

private:
    uint const m_pin{};
};
