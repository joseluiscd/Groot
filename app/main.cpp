#include "application.hpp"
#include <doctest/doctest.h>
#include <bait/host.hpp>

int main(int argc, const char** argv)
{

    bait::create_app_host<Application>();
    return 0;

}

