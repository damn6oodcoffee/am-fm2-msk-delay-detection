
#include <iostream>
#include "Init.hpp"
#include "MainLoop3.hpp"

int main() {
    try {
        auto handler = Init::ImguiAndOpenGLHandler();
        mainLoop3(handler);
    }
    catch (std::exception& e) {
        std::cout << e.what() << '\n';
        return 1;
    }
    catch (...) {
        std::cout << "unexpected exception\n";
        return 1;
    }
}
