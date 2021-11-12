#include "application.hpp"
#include <doctest/doctest.h>

#ifndef NDEBUG
#define CR_DEBUG
#endif

#include <bait/host.hpp>

int main(int argc, const char** argv)
{

    bait::create_app_host<Application>("groot.so");
    return 0;

}

