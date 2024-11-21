
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

    std::future<StatResultDoppler> statResultFuture;
    StatResultDoppler statResult{};
    float statProgress{ -1.0 };
    bool isStatExperimentInProcess{ false };

    ImVec4 blue{ 0.0f, 0.0f, 1.0f, 1.0f };
    ImVec4 orange{ 1.0f, 0.5f, 0.0f, 1.0f };
    ImVec4 green{ 0.0f, 0.8f, 0.0f, 1.0f };
    ImVec4 red{ 1.0f, 0.0f, 0.0f, 1.0f };
    ImVec4 purple{ 0.7f, 0.0f, 1.0f, 1.0f };
    float weight = 2.0f;

    while (handler.windowLoopCondition()) {
        bool skipLoop{ !handler.beginLoopRoutine() };
        if (skipLoop)
            continue;
        // 4. Control window
        ImGui::Begin("Control Window");

        ImGui::SeparatorText((const char*)u8"Параметры сигнала");
        ImGui::PushItemWidth(150.0);
        // Sample Rate Input
        constexpr int bufsize{ 32 };
        static char sampleRateBuf[bufsize] = "100";
        static double sampleRate{};
        ImGui::InputText((const char*)u8"Частота дискретизации (кГц)", sampleRateBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Bit Count Input
        static char bitCountBuf[bufsize] = "80";
        static int bitCount{};
        ImGui::InputText((const char*)u8"Число бит", bitCountBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Bit Rate Input
        static char bitRateBuf[bufsize] = "1600";
        static double bitRate{};
        ImGui::InputText((const char*)u8"Битовая скорость", bitRateBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Carrier Input
        static char carrierBuf[bufsize] = "20";
        static double carrier{};
        ImGui::InputText((const char*)u8"Несущая частота (кГц)", carrierBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Delay Input
        static char delayBuf[bufsize] = "10";
        static double delay{};
        ImGui::InputText((const char*)u8"Задержка по времени (мс)", delayBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Duration Input
        static char durationBuf[bufsize] = "15";
        static double duration{};
        ImGui::InputText((const char*)u8"Длительность опорного сигнала (мс)", durationBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // Doppler Input
        static char dopplerBuf[bufsize] = "500";
        static double doppler{};
        ImGui::InputText((const char*)u8"Доплеровское смещение (Гц)", dopplerBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        // SNR Input
        static char snrBuf[bufsize] = "0";
        static double snr{};
        ImGui::InputText((const char*)u8"ОСШ(дБ)", snrBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);

        // Statistics Input
        static char dopplerStepCountBuf[bufsize] = "40";
        static int dopplerStepCount{};
        static char dopplerLowBuf[bufsize] = "0.0";
        static double dopplerLow{};
        static char dopplerHighBuf[bufsize] = "1000.0";
        static double dopplerHigh{};
        static char repsPerDopplerBuf[bufsize] = "100";
        static int repsPerDoppler{};


        ImGui::SeparatorText((const char*)u8"Модуляция");
        static int modulationType{ 0 };
        ImGui::RadioButton("ASK", &modulationType, 0); ImGui::SameLine();
        ImGui::RadioButton("BPSK", &modulationType, 1); ImGui::SameLine();
        ImGui::RadioButton("MSK", &modulationType, 2);
        static double lowAmp{};
        static char lowAmpBuf[bufsize] = "0.2";
        static double highAmp{};
        static char highAmpBuf[bufsize] = "1.0";
        if (modulationType == 0) {
            ImGui::InputText((const char*)u8"Мин. амплитуда", lowAmpBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);

            ImGui::InputText((const char*)u8"Макс. амплитуда", highAmpBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        }

        ImGui::SeparatorText("");
        if (ImGui::Button((const char*)u8"Сгенерировать")) {
            sampleRate = 1e3 * std::atof(sampleRateBuf); // "1e3 * " - kHz to Hz
            bitCount = std::atoi(bitCountBuf);
            bitRate = std::atof(bitRateBuf);
            carrier = 1e3 * std::atof(carrierBuf); // "1e3 * " - kHz to Hz
            delay = 1e-3 * std::atof(delayBuf); // "1e-3 * " - msec to sec
            duration = 1e-3 * std::atof(durationBuf); // "1e-3 * " - msec to sec
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
            ImGui::InputText((const char*)u8"Оценка сдвина по времени (мс)", delayEstimateBuf, bufsize, ImGuiInputTextFlags_ReadOnly);

        if (ImGui::CollapsingHeader((const char*)u8"Статистика", ImGuiTreeNodeFlags_None)) {
            ImGui::InputText((const char*)u8"Мин. доплеровское смещиние(Гц)", dopplerLowBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
            ImGui::InputText((const char*)u8"Макс. доплеровское смещение(Гц)", dopplerHighBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
            ImGui::InputText((const char*)u8"Число точек в интервале", dopplerStepCountBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
            ImGui::InputText((const char*)u8"Кол-во повторений для каждой точки", repsPerDopplerBuf, bufsize, ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);

            ImGui::BeginDisabled(isStatExperimentInProcess);
            if (ImGui::Button((const char*)u8"Рассчитать статистику")) {
                // Compute Statistics
                sampleRate = 1e3 * std::atof(sampleRateBuf); // "1e3 * " - kHz to Hz
                bitCount = std::atoi(bitCountBuf);
                bitRate = std::atof(bitRateBuf);
                carrier = 1e3 * std::atof(carrierBuf); // "1e3 * " - kHz to Hz
                delay = 1e-3 * std::atof(delayBuf); // "1e-3 * " - msec to sec
                duration = 1e-3 * std::atof(durationBuf); // "1e-3 * " - msec to sec
                snr = std::atof(snrBuf);
                lowAmp = std::atof(lowAmpBuf);
                highAmp = std::atof(highAmpBuf);
                dopplerLow = std::atof(dopplerLowBuf);
                dopplerHigh = std::atof(dopplerHighBuf);
                dopplerStepCount = std::atoi(dopplerStepCountBuf);
                repsPerDoppler = std::atoi(repsPerDopplerBuf);
                statResultFuture = std::async(std::launch::async, statisticalExperiment,
                    lowAmp, highAmp, sampleRate, bitCount, bitRate,
                    carrier, delay, duration, dopplerLow, dopplerHigh, dopplerStepCount,
                    repsPerDoppler, snr, &statProgress);
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

            if (ImPlot::BeginPlot((const char*)u8"Взаимная корреляция")) {
                if (expResult.crossCorrelation.timeSamples.size() != 0 && expResult.crossCorrelation.timeSamples.size() == expResult.crossCorrelation.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.crossCorrelation.timeSamples.size());
                    ImPlot::SetNextLineStyle(blue, weight);
                    ImPlot::PlotLine((const char*)u8"Взаимная корреляция", &expResult.crossCorrelation.timeSamples[0],
                        &expResult.crossCorrelation.valueSamples[0], dataSize);
                    
                    ImPlot::SetNextLineStyle(red, weight);
                    ImPlot::TagX(expResult.estimatedDelay, ImVec4(1.0, 0.0, 0.0, 1.0));
                    ImPlot::PlotInfLines((const char*)u8"Оценка задержки", &expResult.estimatedDelay, 1);

                    ImPlot::SetNextLineStyle(green, weight);
                    ImPlot::TagX(delay, ImVec4(0.0, 1.0, 0.0, 1.0));
                    ImPlot::PlotInfLines((const char*)u8"Истинная задержка", &delay, 1);

                }
                ImPlot::EndPlot();
            }

            if (false && ImPlot::BeginPlot((const char*)u8"Reference Signal")) {
                if (expResult.refSignal.timeSamples.size() != 0 && expResult.refSignal.timeSamples.size() == expResult.refSignal.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refSignal.timeSamples.size());
                    ImPlot::SetNextLineStyle(blue, weight);
                    ImPlot::PlotLine("Reference Signal", &expResult.refSignal.timeSamples[0],
                        &expResult.refSignal.valueSamples[0], dataSize);
                }
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot((const char*)u8"Сигнал")) {
                if (expResult.refSignal.timeSamples.size() != 0 && expResult.refSignal.timeSamples.size() == expResult.refSignal.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refSignal.timeSamples.size());
                    ImPlot::SetNextLineStyle(green, weight);
                    ImPlot::PlotLine((const char*)u8"Опорный сигнал", &expResult.refSignal.timeSamples[0],
                        &expResult.refSignal.valueSamples[0], dataSize);
                }
                if (expResult.delayedSignal.timeSamples.size() != 0 && expResult.delayedSignal.timeSamples.size() == expResult.delayedSignal.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.delayedSignal.timeSamples.size());
                    ImPlot::SetNextLineStyle(blue, weight);
                    ImPlot::PlotLine((const char*)u8"Исследуемый сигнал", &expResult.delayedSignal.timeSamples[0],
                        &expResult.delayedSignal.valueSamples[0], dataSize);
                    if (!expResult.refSignal.timeSamples.empty()) {
                        double region[2] = { expResult.refSignal.timeSamples.front(), expResult.refSignal.timeSamples.back() };
                        ImPlot::SetNextLineStyle(red, weight);
                        ImPlot::PlotInfLines((const char*)u8"Область опорного сигнала", region, 2);
                        ImPlot::TagX(region[0], ImVec4(1.0, 0.0, 0.0, 1.0));
                        ImPlot::TagX(region[1], ImVec4(1.0, 0.0, 0.0, 1.0));
                    }
                }
                ImPlot::EndPlot();
            }


            if (false && ImPlot::BeginPlot((const char*)u8"IQ компоненты")) {
                if (expResult.refI.timeSamples.size() != 0 && expResult.refI.timeSamples.size() == expResult.refI.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refI.timeSamples.size());
                    ImPlot::SetNextLineStyle(green, weight);
                    ImPlot::PlotLine("I", &expResult.refI.timeSamples[0],
                        &expResult.refI.valueSamples[0], dataSize);
                }
                if (expResult.refQ.timeSamples.size() != 0 && expResult.refQ.timeSamples.size() == expResult.refQ.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refQ.timeSamples.size());
                    ImPlot::SetNextLineStyle(purple, weight);
                    ImPlot::PlotLine("Q", &expResult.refQ.timeSamples[0],
                        &expResult.refQ.valueSamples[0], dataSize);
                }
                ImPlot::EndPlot();
            }
            
            if (ImPlot::BeginPlot((const char*)u8"IQ компоненты")) {
                if (expResult.refI.timeSamples.size() != 0 && expResult.refI.timeSamples.size() == expResult.refI.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refI.timeSamples.size());
                    ImPlot::SetNextLineStyle(green, weight);
                    ImPlot::PlotLine((const char*)u8"I опорного сигнала", &expResult.refI.timeSamples[0],
                        &expResult.refI.valueSamples[0], dataSize);
                    //ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
                    //ImPlot::PlotShaded((const char*)u8"I опорного сигнала", &expResult.refI.timeSamples[0], &expResult.refI.valueSamples[0], dataSize, 0.0);
                    //ImPlot::PlotLine((const char*)u8"I опорного сигнала", &expResult.refI.timeSamples[0], &expResult.refI.valueSamples[0], dataSize);
                    //ImPlot::PopStyleVar();
                }
                if (expResult.refQ.timeSamples.size() != 0 && expResult.refQ.timeSamples.size() == expResult.refQ.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.refQ.timeSamples.size());
                    ImPlot::SetNextLineStyle(purple, weight);
                    ImPlot::PlotLine((const char*)u8"Q опорного сигнала", &expResult.refQ.timeSamples[0],
                        &expResult.refQ.valueSamples[0], dataSize);
                    
                    //ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
                    //ImPlot::PlotShaded((const char*)u8"Q опорного сигнала", &expResult.refQ.timeSamples[0], &expResult.refQ.valueSamples[0], dataSize, 0.0);
                    //ImPlot::PlotLine((const char*)u8"Q опорного сигнала", &expResult.refQ.timeSamples[0], &expResult.refQ.valueSamples[0], dataSize);
                    //ImPlot::PopStyleVar();
                }
                if (expResult.I.timeSamples.size() != 0 && expResult.I.timeSamples.size() == expResult.I.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.I.timeSamples.size());
                    ImPlot::SetNextLineStyle(blue, weight);
                    ImPlot::PlotLine("I", &expResult.I.timeSamples[0],
                        &expResult.I.valueSamples[0], dataSize);
                }
                if (expResult.Q.timeSamples.size() != 0 && expResult.Q.timeSamples.size() == expResult.Q.valueSamples.size()) {
                    auto dataSize = static_cast<int>(expResult.Q.timeSamples.size());
                    ImPlot::SetNextLineStyle(orange, weight);
                    ImPlot::PlotLine("Q", &expResult.Q.timeSamples[0],
                        &expResult.Q.valueSamples[0], dataSize);
                }
                if (!expResult.refSignal.timeSamples.empty()) {
                    double region[2] = { expResult.refSignal.timeSamples.front(), expResult.refSignal.timeSamples.back() };
                    ImPlot::SetNextLineStyle(red, weight);
                    ImPlot::PlotInfLines((const char*)u8"Область опорного сигнала", region, 2);
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
            ImGui::Begin((const char*)u8"Статистика");
            if (ImPlot::BeginPlot((const char*)u8"Статистика")) {
                if (statResult.ASKmaxToStd.timeSamples.size() != 0 && statResult.ASKmaxToStd.timeSamples.size() == statResult.ASKmaxToStd.valueSamples.size()) {
                    auto dataSize = static_cast<int>(statResult.ASKmaxToStd.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("ASK", &statResult.ASKmaxToStd.timeSamples[0],
                        &statResult.ASKmaxToStd.valueSamples[0], dataSize);
                }
                if (statResult.BPSKmaxToStd.timeSamples.size() != 0 && statResult.BPSKmaxToStd.timeSamples.size() == statResult.BPSKmaxToStd.valueSamples.size()) {
                    auto dataSize = static_cast<int>(statResult.BPSKmaxToStd.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("BPSK", &statResult.BPSKmaxToStd.timeSamples[0],
                        &statResult.BPSKmaxToStd.valueSamples[0], dataSize);
                }
                if (statResult.MSKmaxToStd.timeSamples.size() != 0 && statResult.MSKmaxToStd.timeSamples.size() == statResult.MSKmaxToStd.valueSamples.size()) {
                    auto dataSize = static_cast<int>(statResult.MSKmaxToStd.timeSamples.size());
                    //ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 1.5f);
                    ImPlot::PlotLine("MSK", &statResult.MSKmaxToStd.timeSamples[0],
                        &statResult.MSKmaxToStd.valueSamples[0], dataSize);
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