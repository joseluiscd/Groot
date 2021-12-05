#include "src/application.hpp"

int main() {
    entt::registry reg;

    Application app(reg);
    app.main_loop();
    return 0;
}