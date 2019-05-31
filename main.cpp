#include <iostream>
#include "MouseCtrl.h"
#include <unistd.h>

using namespace std;

int main()
{
    cout << "Hello World!" << endl;
    CMouseCtrl Mouse;// = CMouseCtrl();
    //CMouseCtrl Mouse = CMouseCtrl();
    CMouseUI UI = CMouseUI(Mouse.messages);

    Mouse.startCtrlThread();

    Mouse.sendNL();
    usleep(10000);

    Mouse.MotorP = 17;
    Mouse.MotorI = 0;
    Mouse.MotorD = 35;

    Mouse.MotorSetup();

    //usleep(200000);
    UI.process();

    return 0;
}
