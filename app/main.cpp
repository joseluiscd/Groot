#include "application.hpp"
#include <doctest/doctest.h>
#include <bait/bait.hpp>

int main(int argc, const char** argv)
{
    /*doctest::Context context;

    context.applyCommandLine(argc, argv);

    int res = context.run(); // run

    if(context.shouldExit()) // important - query flags (and --exit) rely on the user doing this
        exit(res);

    return res;
    */

    bait::create_app<Application>([](Application& app){
        app.main_loop();
    });
    return 0;

}

