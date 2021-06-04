#include "application.hpp"
#include <doctest/doctest.h>

int main(int argc, const char** argv)
{
    doctest::Context context;

    context.applyCommandLine(argc, argv);

    int res = context.run(); // run

    if(context.shouldExit()) // important - query flags (and --exit) rely on the user doing this
        exit(res);

    return res;
}

