
#include "Init.hpp"
#include "MainLoop.hpp"

int main() {
    auto handler = Init::ImguiAndOpenGLHandler();
    mainLoop(handler);
}
