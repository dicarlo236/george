#include "mbed.h"
#include "io.h"
#include "plotter.h"
#include "foc.h"
#include "vehicle_controller.h"


int main() {
    init_plotter(9);
    setup();
    while(true)
    {
        wait(0.1);
        //plot(0, 0.2);
        send_message();
    }
    
}
