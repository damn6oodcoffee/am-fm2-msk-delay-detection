
#include <future>
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "Tasks3.hpp"
#include "MainLoop3.hpp"


void mainLoop3(Init::ImguiAndOpenGLHandler& handler) {
    using namespace Task3;
    bool show_demo_window = true;
    bool show_plot_demo_window = true;

    ExperimentResult expResult{};
    expResult.estimatedDelay = 0.0;

    std::future<StatResult> statResultFuture;
    StatResult statResult{};
    float statProgress{ -1.0 };
    bool isStatExperimentInProcess{ false };

    while (handler.windowLoopCondition()) {
        bool skipLoop{ !handler.beginLoopRoutine() };
        if (skipLoop)
            continue;
        // 4. Control window
        ImGui::Begin("Control Window");

        ImGui::SeparatorText("General");
        ImGui::PushItemWidth(150.0);
        // Sample Rate Input
        constexpr int bufsize{ 32 };
        static char sampleRateBuf[bufsize] = "0.5";
        static double sampleRate{};
        ImGui::InputText((const char*)u8"õåõåõå Sample Rate(kHz)", sampleRateBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Bit Count Input
        static char bitCountBuf[bufsize] = "40";
        static int bitCount{};
        ImGui::InputText("Bit Count", bitCountBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Bit Rate Input
        static char bitRateBuf[bufsize] = "80";
        static double bitRate{};
        ImGui::InputText("Bit Rate", bitRateBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Carrier Input
        static char carrierBuf[bufsize] = "0.05";
        static double carrier{};
        ImGui::InputText("Carrier(kHz)", carrierBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Delay Input
        static char delayBuf[bufsize] = "2130";
        static double delay{};
        ImGui::InputText("Delay(ms)", delayBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Duration Input
        static char durationBuf[bufsize] = "1000";
        static double duration{};
        ImGui::InputText("Duration(ms)", durationBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Doppler Input
        static char dopplerBuf[bufsize] = "500";
        static double doppler{};
        ImGui::InputText("Doppler Shift(Hz)", dopplerBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
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


        ImGui::SeparatorText("Modulation");
        static int modulationType{ 0 };
        ImGui::RadioButton("ASK", &modulationType, 0); ImGui::SameLine();
        ImGui::RadioButton("BPSK", &modulationType, 1); ImGui::SameLine();
        ImGui::RadioButton("MSK", &modulationType, 2);
        static double lowAmp{};
        static char lowAmpBuf[bufsize] = "0.0";
        static double highAmp{};
        static char highAmpBuf[bufsize] = "1.0";
        if (modulationType == 0) {
            ImGui::InputText("Low Amplitude", lowAmpBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);

            ImGui::InputText("High Amplitude", highAmpBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        }

        ImGui::SeparatorText("");
        if (ImGui::Button("Generate")) {
            sampleRate = 1e3 * std::atof(sampleRateBuf); // "1e3 * " - kHz to Hz
            bitCount = std::atoi(bitCountBuf);
            bitRate = std::atof(bitRateBuf);
            carrier = 1e3 * std::atof(carrierBuf); // "1e3 * " - kHz to Hz
            delay = 1e-3 * std::atof(delayBuf); // "1e-3 * " - msec to sec
            duration = 1e-3 * std::atof(dopplerBuf); // "1e-3 * " - msec to sec
            doppler = std::atof(dopplerBuf);
            snr = std::atof(snrBuf);
            lowAmp = std::atof(lowAmpBuf);
            highAmp = std::atof(highAmpBuf);
            if (modulationType == 0)
                expResult = singleExperimentASK(lowAmp, highAmp, sampleRate, bitCount, bitRate, carrier, delay, duration, snr, doppler);
            if (modulationType == 1)
                expResult = singleExperimentBPSK(sampleRate, bitCount, bitRate, carrier, delay, duration, snr, doppler);
            if (modulationType == 2)
                expResult = singleExperimentMSK(sampleRate, bitCount, bitRate, carrier, delay, duration, snr, doppler);
        }
        ImGui::SameLine();
        static char delayEstimateBuf[bufsize];
        if (std::snprintf(delayEstimateBuf, bufsize, "%f", 1e3 * expResult.estimatedDelay) > 0)
            ImGui::InputText("Delay Estimate (ms)", delayEstimateBuf, bufsize, ImGuiInputTextFlags_ReadOnly);

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
                carrier = 1e3 * std::atof(carrierBuf); // "1e3 * " - kHz to Hz
                delay = 1e-3 * std::atof(delayBuf); // "1e-3 * " - msec to sec
                snr = std::atof(snrBuf);
                lowAmp = std::atof(lowAmpBuf);
                highAmp = std::atof(highAmpBuf);
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

        ImGui::End();

        // 5. ImPlot window for signals.
        {
            ImGui::Begin("Plots");

            if (ImPlot::BeginPlot("Cross-correlation")) {
                if (expResult.crossCorrelation.timeSamples.size() != 0 && expResult.crossCorrelation.timeSamples.size() == expResult.crossCorrelation.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.crossCorrelation.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("Cross-correlation", &expResult.crossCorrelation.timeSamples[0],
                        &expResult.crossCorrelation.valueSamples[0], dataSize);

                    ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::TagX(expResult.estimatedDelay, ImVec4(1.0, 0.0, 0.0, 1.0));
                    ImPlot::PlotInfLines("Estimated Delay", &expResult.estimatedDelay, 1);

                    ImPlot::SetNextLineStyle(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::TagX(delay, ImVec4(0.0, 1.0, 0.0, 1.0));
                    ImPlot::PlotInfLines("True Delay", &delay, 1);

                }
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Reference Signal")) {
                if (expResult.refSignal.timeSamples.size() != 0 && expResult.refSignal.timeSamples.size() == expResult.refSignal.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refSignal.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("Reference Signal", &expResult.refSignal.timeSamples[0],
                        &expResult.refSignal.valueSamples[0], dataSize);
                }
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Reference IQ")) {
                if (expResult.refI.timeSamples.size() != 0 && expResult.refI.timeSamples.size() == expResult.refI.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refI.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("I", &expResult.refI.timeSamples[0],
                        &expResult.refI.valueSamples[0], dataSize);
                }
                if (expResult.refQ.timeSamples.size() != 0 && expResult.refQ.timeSamples.size() == expResult.refQ.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refQ.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("Q", &expResult.refQ.timeSamples[0],
                        &expResult.refQ.valueSamples[0], dataSize);
                }
                ImPlot::EndPlot();
            }
            
            if (ImPlot::BeginPlot("Analyzed IQ")) {
                if (expResult.I.timeSamples.size() != 0 && expResult.I.timeSamples.size() == expResult.I.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.I.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("I", &expResult.I.timeSamples[0],
                        &expResult.I.valueSamples[0], dataSize);
                }
                if (expResult.Q.timeSamples.size() != 0 && expResult.Q.timeSamples.size() == expResult.Q.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.Q.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("Q", &expResult.Q.timeSamples[0],
                        &expResult.Q.valueSamples[0], dataSize);
                }
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Delayed Signal")) {
                if (expResult.delayedSignal.timeSamples.size() != 0 && expResult.delayedSignal.timeSamples.size() == expResult.delayedSignal.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.delayedSignal.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("Delayed Signal", &expResult.delayedSignal.timeSamples[0],
                        &expResult.delayedSignal.valueSamples[0], dataSize);
                    double region[2] = { expResult.refSignal.timeSamples.front(), expResult.refSignal.timeSamples.back() };
                    ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotInfLines("Ref Signal Region", region, 2);
                    ImPlot::TagX(region[0], ImVec4(1.0, 0.0, 0.0, 1.0));
                    ImPlot::TagX(region[1], ImVec4(1.0, 0.0, 0.0, 1.0));
                }
                ImPlot::EndPlot();
            }

            ImGui::End();
        }

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
        
        if (show_demo_window)
            ImGui::ShowDemoWindow();
        if (show_plot_demo_window)
            ImPlot::ShowDemoWindow();
        
        handler.endLoopRoutine();
    }
}