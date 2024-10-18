

#include <iostream>
#include <future>
#include <vector>
#include <filesystem>
#include <cstdio>
#include <glad/glad.h>
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "SignalModel.hpp"
#include "Tasks2.hpp"

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int main() {

#if 1
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1280, 720, "Dear ImGui GLFW+OpenGL3 example", nullptr, nullptr);
    if (window == nullptr)
        return -1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

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
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    std::filesystem::path cwd = std::filesystem::current_path();
    std::cout << "current path: " << cwd.string() << std::endl;
    auto fontPath = std::filesystem::path(cwd.string() + "/../imgui/misc/fonts/Roboto-Medium.ttf");
    fontPath.make_preferred();
    std::cout << "font path: " << fontPath << std::endl;
    // Load font
    io.Fonts->AddFontFromFileTTF(
        fontPath.string().c_str(),
        18
    );
#endif

#if 0
    ExperimentResult expResult{};
    
    std::future<StatResult> statResultFuture;
    StatResult statResult{};
    float statProgress{ -1.0 };
    bool isStatExperimentInProcess{ false };
#endif
    
	QPSKGoldCodeExperiment::ExperimentResult expResult;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

#if 0
        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        if (show_demo_window && false)
            ImGui::ShowDemoWindow(&show_demo_window);

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

            ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
            ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
            ImGui::Checkbox("Another Window", &show_another_window);

            ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

            if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::End();
        }
        // 3. Show another simple window.
        if (show_another_window) {
            ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
            ImGui::Text("Hello from another window!");
            if (ImGui::Button("Close Me"))
                show_another_window = false;
            ImGui::End();
        }
#endif
        // 4. Control window

        ImGui::Begin("Control Window");

        ImGui::SeparatorText("General");
        ImGui::PushItemWidth(150.0);
        // Sample Rate Input
        constexpr int bufsize{ 32 };
        static char sampleRateBuf[bufsize] = "0.5";
        static double sampleRate{};
        ImGui::InputText("Sample Rate(kHz)", sampleRateBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Bit Count Input
        static char bitCountBuf[bufsize] = "40";
        static int bitCount{};
        ImGui::InputText("Bit Count", bitCountBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Bit Rate Input
        static char bitRateBuf[bufsize] = "80";
        static double bitRate{};
        ImGui::InputText("Bit Rate", bitRateBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // SNR Input
        static char snrBuf[bufsize] = "0";
        static double snr{};
        ImGui::InputText("SNR(dB)", snrBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);

        // Statistics Input
        static char snrStepCountBuf[bufsize] = "20";
        static int snrStepCount{};
        static char snrLowBuf[bufsize] = "-10.0";
        static double snrLow{};
        static char snrHighBuf[bufsize] = "10.0";
        static double snrHigh{};
        static char repsPerSNRBuf[bufsize] = "1000";
        static int repsPerSNR{};

        ImGui::SeparatorText("");
        if (ImGui::Button("Generate")) {
            sampleRate = 1e3 * std::atof(sampleRateBuf); // "1e3 * " - kHz to Hz
            bitCount = std::atoi(bitCountBuf);
            bitRate = std::atof(bitRateBuf);
            snr = std::atof(snrBuf);
            QPSKGoldCodeExperiment gcExp(sampleRate, bitRate);
            expResult = gcExp.doExperiment(bitCount, snr);
        }
#if 0
        if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_None)) {
            ImGui::InputText("SNR Low(dB)", snrLowBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
            ImGui::InputText("SNR High(dB)", snrHighBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
            ImGui::InputText("SNR Step Count", snrStepCountBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
            ImGui::InputText("Repetitions Per SNR", repsPerSNRBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);

            ImGui::BeginDisabled(isStatExperimentInProcess);
            if (ImGui::Button("Get Stats")) {
                // Compute Statistics
                sampleRate = 1e3 * std::atof(sampleRateBuf); // "1e3 * " - kHz to Hz
                bitCount = std::atoi(bitCountBuf);
                bitRate = std::atof(bitRateBuf);
                snr = std::atof(snrBuf);
                snrLow = std::atof(snrLowBuf);
                snrHigh = std::atof(snrHighBuf);
                snrStepCount = std::atoi(snrStepCountBuf);
                repsPerSNR = std::atoi(repsPerSNRBuf);
                statResultFuture = std::async(std::launch::async, statisticalExperiment,
                    lowAmp, highAmp, sampleRate, bitCount, bitRate,
                    carrier, delay, snrLow, snrHigh, snrStepCount,
                    repsPerSNR, &statProgress);
                isStatExperimentInProcess = true;
            }
            ImGui::EndDisabled();
            ImGui::ProgressBar(statProgress, ImVec2(-1.0, 0.0));
            ImGui::PopItemWidth();
        }
#endif
        ImGui::End();

        // 5. ImPlot window for signals.
        {
            ImGui::Begin("Plots");

            if (ImPlot::BeginPlot("I")) {
                if (expResult.I.timeSamples.size() != 0 && expResult.I.timeSamples.size() == expResult.I.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.I.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("I", &expResult.I.timeSamples[0],
                        &expResult.I.valueSamples[0], dataSize);
                }
#if 0
                if (expResult.bitSamples.timeSamples.size() != 0 && expResult.bitSamples.timeSamples.size() == expResult.bitSamples.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.bitSamples.timeSamples.size());
                    ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
                    ImPlot::PlotShaded("Bits", &expResult.bitSamples.timeSamples[0], &expResult.bitSamples.valueSamples[0], dataSize, 0.0);
                    ImPlot::PlotLine("Bits", &expResult.bitSamples.timeSamples[0], &expResult.bitSamples.valueSamples[0], dataSize, 0.0);
                    ImPlot::PopStyleVar();
                }
#endif
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot("Q")) {
                if (expResult.Q.timeSamples.size() != 0 && expResult.Q.timeSamples.size() == expResult.Q.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.Q.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("Q", &expResult.Q.timeSamples[0],
                        &expResult.Q.valueSamples[0], dataSize);
                }
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot("Filter Output")) {
                for (int i{ 0 }; i < QPSKGoldCodeExperiment::filterCount; ++i) {
					if (expResult.filterConv[i].timeSamples.size() != 0 && 
                        expResult.filterConv[i].timeSamples.size() == expResult.filterConv[i].valueSamples.size()) {
						auto dataSize = static_cast<int>(expResult.filterConv[i].timeSamples.size());
						//ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                        char legendName[16];
                        sprintf_s(legendName, 16, "Filter %d", i);
						ImPlot::PlotLine(legendName, &expResult.filterConv[i].timeSamples[0],
							&expResult.filterConv[i].valueSamples[0], dataSize);
					}
                }
                ImPlot::EndPlot();
            }

            ImGui::End();
        }

#if 0
        // Check if stat result is ready
        {
            if (statResultFuture.valid()) {
                auto status = statResultFuture.wait_for(std::chrono::seconds(0));
                if (status == std::future_status::ready) {
                    statResult = statResultFuture.get();
                    isStatExperimentInProcess = false;
                    statProgress = -1.0;
                }
            }
        }
        // 6. ImPlot window for stat data
        {
            ImGui::Begin("Statistics");
            if (ImPlot::BeginPlot("Statistics")) {
                if (statResult.ASKprobVsSNR.timeSamples.size() != 0 && statResult.ASKprobVsSNR.timeSamples.size() == statResult.ASKprobVsSNR.valueSamples.size()) {
                    auto dataSize = static_cast<int>(statResult.ASKprobVsSNR.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("ASK", &statResult.ASKprobVsSNR.timeSamples[0],
                        &statResult.ASKprobVsSNR.valueSamples[0], dataSize);
                }
                if (statResult.BPSKprobVsSNR.timeSamples.size() != 0 && statResult.BPSKprobVsSNR.timeSamples.size() == statResult.BPSKprobVsSNR.valueSamples.size()) {
                    auto dataSize = static_cast<int>(statResult.BPSKprobVsSNR.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("BPSK", &statResult.BPSKprobVsSNR.timeSamples[0],
                        &statResult.BPSKprobVsSNR.valueSamples[0], dataSize);
                }
                if (statResult.MSKprobVsSNR.timeSamples.size() != 0 && statResult.MSKprobVsSNR.timeSamples.size() == statResult.MSKprobVsSNR.valueSamples.size()) {
                    auto dataSize = static_cast<int>(statResult.MSKprobVsSNR.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("MSK", &statResult.MSKprobVsSNR.timeSamples[0],
                        &statResult.MSKprobVsSNR.valueSamples[0], dataSize);
                }
                ImPlot::EndPlot();
            }
            ImGui::End();
        }
#endif
        //ImGui::ShowDemoWindow();
        ImPlot::ShowDemoWindow();

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
