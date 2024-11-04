
#ifndef INIT_HPP
#define INIT_HPP 

#include <memory>
#include <GLFW/glfw3.h> 

namespace Init {

    namespace detail {
        class glfwWindowHandler {
        public:
            glfwWindowHandler(int width, int height, const char* title, GLFWmonitor* monitor, GLFWwindow* share);
            ~glfwWindowHandler();
            void makeCurrent();
            void setUpGlad();
            GLFWwindow* getWindowPtr() { return window; }
        private:
            GLFWwindow* window;
        };
    }
    class ImguiAndOpenGLHandler {
    public:
        ImguiAndOpenGLHandler();
        ~ImguiAndOpenGLHandler();
        ImguiAndOpenGLHandler(const ImguiAndOpenGLHandler&) = delete;
        ImguiAndOpenGLHandler& operator=(const ImguiAndOpenGLHandler&) = delete;
        ImguiAndOpenGLHandler(const ImguiAndOpenGLHandler&&) = delete;
        ImguiAndOpenGLHandler& operator=(const ImguiAndOpenGLHandler&&) = delete;
        bool windowLoopCondition() {
            return !glfwWindowShouldClose(pWindowHandler->getWindowPtr());
        }
        bool beginLoopRoutine();
        void endLoopRoutine();

    private:
        std::unique_ptr<detail::glfwWindowHandler> pWindowHandler;
    };

}
#endif