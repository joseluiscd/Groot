#include "application.hpp"
#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

int main()
{
    Application app;
    app.main_loop();
}
