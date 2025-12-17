#ifndef ARRC_TENKAI_H_
#define ARRC_TENKAI_H_

#include "mbed.h"
#include "robomaster_can.hpp"
#include "pid.hpp"
#include "config.h"
#include <cstdint>

class Tenkai {
private:
    double MAX_CURRENT=8000;

    int16_t clamp_current(double value, int16_t max_val) {
        if (value > max_val) return max_val;
        if (value < -max_val) return -max_val;
        return static_cast<int16_t>(value);
    }

    robomaster::Robomaster_Array* array;   // CANバス
    robomaster::Robomaster_ESC* ESC;       // ESC
    DigitalIn* lim;                       // リミットスイッチ
    PID pid;                              // PID

    
    void robomasmove(int del_angle, double kp, double ki, double kd, int n=0) {
        PID pid(kp, ki, kd);

        int16_t initial_raw = ESC->get_continuous_angle();
        double initial_angle = (initial_raw / (8191.0 * 36)) * 360.0;
        double target = initial_angle + del_angle;

        double angle = (ESC->get_continuous_angle() / (8191.0 * 36)) * 360.0;
        pid.Input(target, angle);
        double output = pid.Output();
        int16_t current = clamp_current(output, MAX_CURRENT);

        ESC->set_current(current);

        if (n != 0) {
            if (lim && lim->read() == 0) {
                ESC->set_current(0);
            }
        }

        array->send();
    }

public:
    
    Tenkai(robomaster::Robomaster_Array* array, 
           robomaster::Robomaster_ESC* esc,
           DigitalIn* lim = nullptr)      //宣言
        :array(array),
          ESC(esc),
          lim(lim),
          pid(0.0, 0.0, 0.0)
    {
        array->add_ESC(ESC);
    }

    void open_robot() {                       //開く
        robomasmove(720, 3.0, 0.0, 0.0,3);
    }

    void close_robot() {                      //閉じる
        robomasmove(-500, 2.0, 0.0, 0.0);
    }
};

#endif
