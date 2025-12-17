一号機展開の使い方
```c++:main.cpp
#include "mbed.h"
#include "robomaster_can.hpp"
#include "pid.hpp"
#include "config.h" 
#include "tenkai.hpp"

using namespace robomaster;

Robomaster_Array array(PB_5, PB_6);  // CAN RX, TX
Robomaster_ESC esc(1);　            // ESC ID   
DigitalIn lim(PA_0);                //展開用リミット

Tenkai tenkai(&array, &esc, &lim);//展開のための宣言

int main() {
    
    tenkai.open_robot();　　　　　　//開く
    wait_us(1000000);
    tenkai.close_robot();　　　　　//閉じる
}
```

