#include "application.hpp"
#include "entt/entity/fwd.hpp"

int main() {
    entt::registry reg;

    Application app(reg);
    app.main_loop();
    return 0;
}