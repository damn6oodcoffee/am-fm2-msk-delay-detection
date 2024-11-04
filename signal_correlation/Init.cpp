
#include <filesystem>
#include <cstdio>
#include <glad/glad.h>
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "Init.hpp"

using namespace Init;

namespace Init::detail {
    void glfwErrorCallback(int error, const char* description) {
        fprintf(stderr, "GLFW Error %d: %s\n", error, description);
    }

    glfwWindowHandler::glfwWindowHandler(int width, int height, const char* title, GLFWmonitor* monitor, GLFWwindow* share) {
        glfwSetErrorCallback(glfwErrorCallback);
        if (!glfwInit())
            throw std::runtime_error("glfw: init failed\n");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        window = glfwCreateWindow(width, height, title, monitor, share);
        if (window == nullptr)
            throw std::runtime_error("glfw: couldn't create window\n");
    }

    glfwWindowHandler::~glfwWindowHandler() {
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    void glfwWindowHandler::makeCurrent() {
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1); // Enable vsync
    }

    void glfwWindowHandler::setUpGlad() {
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            throw std::runtime_error("Failed to initialize GLAD\n");
        }
    }
}

ImguiAndOpenGLHandler::ImguiAndOpenGLHandler() {
    
    pWindowHandler = std::make_unique<detail::glfwWindowHandler>(1280, 720, "Dear ImGui GLFW+OpenGL3 example", nullptr, nullptr);
    pWindowHandler->makeCurrent();
    pWindowHandler->setUpGlad();
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    //ImGui::StyleColorsDark();
    ImGui::StyleColorsLight();


    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(pWindowHandler->getWindowPtr(), true);
    ImGui_ImplOpenGL3_Init();

    std::filesystem::path cwd = std::filesystem::current_path();
    //std::cout << "current path: " << cwd.string() << std::endl;
    auto fontPath = std::filesystem::path(cwd.string() + "/../imgui/misc/fonts/Roboto-Medium.ttf");
    fontPath.make_preferred();
    //std::cout << "font path: " << fontPath << std::endl;
    // Load font
    io.Fonts->AddFontFromFileTTF(
        fontPath.string().c_str(),
        18,
        nullptr,
        io.Fonts->GetGlyphRangesCyrillic()
    );
}

ImguiAndOpenGLHandler::~ImguiAndOpenGLHandler() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

bool ImguiAndOpenGLHandler::beginLoopRoutine() {
    glfwPollEvents();
    if (glfwGetWindowAttrib(pWindowHandler->getWindowPtr(), GLFW_ICONIFIED) != 0) {
        ImGui_ImplGlfw_Sleep(10);
        return false;
    }
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    return true;
}

void ImguiAndOpenGLHandler::endLoopRoutine() {
    // Rendering
    ImGui::Render();
    int display_w, display_h;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    glfwGetFramebufferSize(pWindowHandler->getWindowPtr(), &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(pWindowHandler->getWindowPtr());
}
