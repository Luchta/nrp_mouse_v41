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

    //usleep(200000);
    UI.process();

    //mouse.startUART();
    //mouse.mainCtrl();

    return 0;
}
