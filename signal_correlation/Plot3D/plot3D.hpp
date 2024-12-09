#ifndef PLOT3D_HPP
#define PLOT3D_HPP

#include <vector>
#include <sstream>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "surface.hpp"
#include "shader.hpp"
#include "text_render.hpp"
#include "nice_range.hpp"

inline std::string tickLabelToString(double label, int precision) {
    std::stringstream labelFormatter;
    labelFormatter.flags(std::ios_base::fixed);
    labelFormatter.precision(precision);
    labelFormatter << label; 
    return labelFormatter.str();
}

class PlotGridRenderer {
public:
    PlotGridRenderer() {
        glGenVertexArrays(1, &GridVAO);
        glGenBuffers(1, &GridVBO);
        glGenVertexArrays(1, &EdgeVAO);
        glGenBuffers(1, &EdgeVBO);
        setUpEdge();
    }

    ~PlotGridRenderer() {
        glDeleteVertexArrays(1, &GridVAO);
        glDeleteBuffers(1, &GridVBO);
        glDeleteVertexArrays(1, &EdgeVAO);
        glDeleteBuffers(1, &EdgeVBO);
    }

    void setUpGrid(int xLineCount, int yLineCount, int zLineCount) {
        std::vector<float> newGridVertices;
        float shift { 1.0f / (xLineCount - 1) };
        for (int i { 1 }; i < xLineCount - 1; ++i) {
            newGridVertices.insert(newGridVertices.end(), { 0.0f, 0.0f, shift * i, 1.0f, 0.0f, shift * i });
            newGridVertices.insert(newGridVertices.end(), { 0.0f, 0.0f, shift * i, 0.0f, 1.0f, shift * i });
        }
        shift = 1.0f / (yLineCount - 1);
        for (int i { 1 }; i < yLineCount - 1; ++i) {
            newGridVertices.insert(newGridVertices.end(), { shift * i, 0.0f, 0.0f, shift * i, 0.0f, 1.0f });
            newGridVertices.insert(newGridVertices.end(), { shift * i, 0.0f, 0.0f, shift * i, 1.0f, 0.0f });
        }
        shift = 1.0f / (zLineCount - 1);
        for (int i { 1 }; i < zLineCount - 1; ++i) {
            newGridVertices.insert(newGridVertices.end(), { 0.0f, shift * i, 0.0f, 1.0f, shift * i, 0.0f });
            newGridVertices.insert(newGridVertices.end(), { 0.0f, shift * i, 0.0f, 0.0f, shift * i, 1.0f });
        }
        gridVertices = std::move(newGridVertices);
        bind();
    }

    void bind() {
        glBindVertexArray(EdgeVAO);
        glBindBuffer(GL_ARRAY_BUFFER, EdgeVBO);
        glBufferData(GL_ARRAY_BUFFER, edgeVertices.size() * sizeof(float), &edgeVertices[0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);

        glBindVertexArray(GridVAO);
        glBindBuffer(GL_ARRAY_BUFFER, GridVBO);
        glBufferData(GL_ARRAY_BUFFER, gridVertices.size() * sizeof(float), &gridVertices[0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);

        glBindVertexArray(0);
    }

    void render(Shader& gridShader) {
        gridShader.use();
        glBindVertexArray(EdgeVAO);
        gridShader.setVec3("lineColor", glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
        glDrawArrays(GL_LINES, 0, edgeVertices.size() / 3);
        glBindVertexArray(GridVAO);
        gridShader.setVec3("lineColor", glm::value_ptr(glm::vec3(0.75f, 0.75f, 0.75f)));
        glDrawArrays(GL_LINES, 0, gridVertices.size() / 3);
        glBindVertexArray(0);
    }

private:
    unsigned int GridVAO, GridVBO;
    unsigned int EdgeVAO, EdgeVBO;
    std::vector<float> gridVertices;
    std::vector<float> edgeVertices;

    void setUpEdge() {
        // from origin
        edgeVertices.insert(edgeVertices.end(), { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f });
        edgeVertices.insert(edgeVertices.end(), { 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f });
        edgeVertices.insert(edgeVertices.end(), { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f });
        // xy plane
        edgeVertices.insert(edgeVertices.end(), { 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f });
        edgeVertices.insert(edgeVertices.end(), { 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f });
        // zy plane
        edgeVertices.insert(edgeVertices.end(), { 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f });
        edgeVertices.insert(edgeVertices.end(), { 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f });
        // zx plane
        edgeVertices.insert(edgeVertices.end(), { 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f });
        edgeVertices.insert(edgeVertices.end(), { 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f });
    }

};

class AxisLabels {
public:
    AxisLabels(const std::string& xAxisLabel, const std::string& yAxisLabel, const std::string& zAxisLabel)
        : xAxisName { xAxisLabel }
        , yAxisName { yAxisLabel }
        , zAxisName { zAxisLabel } {};

    std::string xAxisName;
    std::string yAxisName;
    std::string zAxisName;

    void setTickLabels(const RangeBox& rangeBox) {
        std::vector<double> newXTickLabels, newYTickLabels, newZTickLabels;
        auto xRange { getNiceRange(rangeBox.xBegin, rangeBox.xEnd, 5) };
        auto yRange { getNiceRange(rangeBox.yBegin, rangeBox.yEnd, 5) };
        auto zRange { getNiceRange(rangeBox.zBegin, rangeBox.zEnd, 5) };
        for (double x { xRange.startTick }, xEnd { xRange.endTick + 0.5*xRange.tickWidth }; x < xEnd; x += xRange.tickWidth) 
            newXTickLabels.push_back(x); 
        for (double y { yRange.startTick }, yEnd { yRange.endTick + 0.5*yRange.tickWidth }; y < yEnd; y += yRange.tickWidth)
            newYTickLabels.push_back(y);
        for (double z { zRange.startTick }, zEnd { zRange.endTick + 0.5*zRange.tickWidth }; z < zEnd; z += zRange.tickWidth) 
            newZTickLabels.push_back(z);
        std::cout << "ticks: " << newXTickLabels.size() << "\n";
        std::cout << "startTick: " << xRange.startTick << "\n";
        std::cout << "endTick: " << xRange.endTick << "\n";
        std::cout << "tickWidth: " << xRange.tickWidth << "\n";

        xPrecision = xRange.fractionPrecision;
        yPrecision = yRange.fractionPrecision;
        zPrecision = zRange.fractionPrecision;
        xTickLabels = std::move(newXTickLabels);
        yTickLabels = std::move(newYTickLabels);
        zTickLabels = std::move(newZTickLabels);
        niceRangeBox = { 
            xRange.startTick, xRange.endTick,
            yRange.startTick, yRange.endTick,
            zRange.startTick, zRange.endTick,
        };
    }

    const std::vector<double>& getTickLabelsX() { return xTickLabels; }
    const std::vector<double>& getTickLabelsY() { return yTickLabels; }
    const std::vector<double>& getTickLabelsZ() { return zTickLabels; }
    int getPrecisionX() { return xPrecision; }
    int getPrecisionY() { return yPrecision; }
    int getPrecisionZ() { return zPrecision; }
    const RangeBox& getRange() { return niceRangeBox; }
    
private:
    std::vector<double> xTickLabels;
    std::vector<double> yTickLabels;
    std::vector<double> zTickLabels;
    int xPrecision;
    int yPrecision;
    int zPrecision;
    RangeBox niceRangeBox;
};

class AxisLabelsRenderer {
public:
    AxisLabelsRenderer(const std::string& fontName, const std::string& xAxisLabel = "xLabel Wow!", 
                 const std::string& yAxisLabel = "y", const std::string& zAxisLabel = "z")
        : textRenderer(fontName)
        , axes(xAxisLabel, yAxisLabel, zAxisLabel) {};

    void setTickLabels(const RangeBox& rangeBox) {
        axes.setTickLabels(rangeBox);
    }

    void bind() {
        textRenderer.bind();
    }

    void render(Shader& textShader) {
        textShader.use();
        glm::mat4 identityMat { glm::mat4(1.0f) };
        glm::mat4 model(1.0f);
        model = glm::translate(identityMat, glm::vec3(1.25f, 0.0f, 0.5f));
        model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));  
        textShader.setMat4("model", glm::value_ptr(model));
        textRenderer.render(axes.xAxisName, 0.0f, 0.0f, 0.002f);

        model = glm::translate(identityMat, glm::vec3(0.5f, 0.0f, 1.25f));
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));  
        textShader.setMat4("model", glm::value_ptr(model));
        textRenderer.render(axes.yAxisName, 0.0f, 0.0f, 0.002f);

        model = glm::translate(identityMat, glm::vec3(-0.07f, 0.5f, 1.07f));
        model = glm::rotate(model, glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f)); 
        model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        textShader.setMat4("model", glm::value_ptr(model));
        textRenderer.render(axes.zAxisName, 0.0f, 0.0f, 0.002f);

        auto tickLabels = axes.getTickLabelsX();
        double shift { 1.0 / static_cast<double>(tickLabels.size() - 1) };
        double pos { 1.0 };
        for (auto&& x: tickLabels) {
            auto label = tickLabelToString(x, axes.getPrecisionX());
            model = glm::translate(identityMat, glm::vec3(1.1f, 0.0f, pos));
            model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));   
            textShader.setMat4("model", glm::value_ptr(model));
            textRenderer.render(label, 0.0f, 0.0f, 0.001f);
            pos -= shift;
        }

        tickLabels = axes.getTickLabelsY();
        shift = 1.0 / static_cast<double>(tickLabels.size() - 1);
        pos = 1.0;
        for (auto&& y: tickLabels) {
            auto label = tickLabelToString(y, axes.getPrecisionY());
            model = glm::translate(identityMat, glm::vec3(pos, 0.0f, 1.1f));
            model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));  
            textShader.setMat4("model", glm::value_ptr(model));
            textRenderer.render(label, 0.0f, 0.0f, 0.001f);
            pos -= shift;
        }

        tickLabels = axes.getTickLabelsZ();
        shift = 1.0 / static_cast<double>(tickLabels.size() - 1);
        pos = 0.0;
        for (auto&& z: tickLabels) {
            auto label = tickLabelToString(z, axes.getPrecisionZ());
            model = glm::translate(identityMat, glm::vec3(0.0f, pos, 1.0f));
            model = glm::rotate(model, glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f)); 
            textShader.setMat4("model", glm::value_ptr(model));
            textRenderer.render(label, 0.0f, 0.0f, 0.001f);
            pos += shift;
        }
        bind();
    }
    
    RangeBox getRange() {
        return axes.getRange();
    }

    int getTickCountX() {
        return static_cast<int>(axes.getTickLabelsX().size());
    }

    int getTickCountY() {
        return static_cast<int>(axes.getTickLabelsY().size());
    }

    int getTickCountZ() {
        return static_cast<int>(axes.getTickLabelsZ().size());
    }

private:
    TextRenderer textRenderer;
    AxisLabels axes;
};


#if 1
class Plot3D {
public:
    Plot3D(const std::string& fontName)
        : surfaceRenderer()
        , axesRenderer(fontName, "delay(sec)" ,"doppler(Hz)", "z")
        , plotGridRenderer()
        , fitSurfaceMatrix(1.0f)
    {};

    // void setColormap(ColormapFunc colormapFunc) {
    //     surfaceRenderer.setColormap(colormapFunc);
    // }

    void plot(const Surface3D& surface, ColormapFunc colormapFunc = viridisColormap, CoordsColoringRule coloringRule = defaultColoringRule) {
        surfaceRenderer.plot(surface, colormapFunc, coloringRule);
        axesRenderer.setTickLabels(surface.getRangeBox());
        plotGridRenderer.setUpGrid(axesRenderer.getTickCountX(), axesRenderer.getTickCountY(), axesRenderer.getTickCountZ());
        //plotGridRenderer.setUpGrid(5,5,5);
        fitSurface(surface.getRangeBox());
        bind();
    }

    void bind() {
        surfaceRenderer.bind();
        axesRenderer.bind();
        plotGridRenderer.bind();
    }

    void render(Shader& gridShader, Shader& axisShader, Shader& surfaceShader) {
        plotGridRenderer.render(gridShader);
        axesRenderer.render(axisShader);
        surfaceShader.use();
        glm::mat4 model{ fitSurfaceMatrix };

        model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        surfaceShader.setMat4("model", glm::value_ptr(model));
        surfaceRenderer.render();
    }

private:
    SurfaceRenderer surfaceRenderer;
    AxisLabelsRenderer axesRenderer;
    PlotGridRenderer plotGridRenderer;
    glm::mat4 fitSurfaceMatrix;
    void fitSurface(RangeBox surfaceRange) {
        double centerX { (surfaceRange.xEnd + surfaceRange.xBegin) / 2 };
        double centerY { (surfaceRange.yEnd + surfaceRange.yBegin) / 2 };
        double centerZ { (surfaceRange.zEnd + surfaceRange.zBegin) / 2 };
        auto niceRange { axesRenderer.getRange() };
        double scaleX { (surfaceRange.xEnd - surfaceRange.xBegin) / ((surfaceRange.xEnd - surfaceRange.xBegin) * (niceRange.xEnd - niceRange.xBegin)) };
        double scaleY { (surfaceRange.yEnd - surfaceRange.yBegin) / ((surfaceRange.yEnd - surfaceRange.yBegin) * (niceRange.yEnd - niceRange.yBegin)) };
        double scaleZ { (surfaceRange.zEnd - surfaceRange.zBegin) / ((surfaceRange.zEnd - surfaceRange.zBegin) * (niceRange.zEnd - niceRange.zBegin)) };
        double translateX { 1.0 - 1*(centerX - niceRange.xBegin) / (niceRange.xEnd - niceRange.xBegin) };
        double translateY { 1.0 - 1*(centerY - niceRange.yBegin) / (niceRange.yEnd - niceRange.yBegin) };
        double translateZ { 1.0 - 1*(centerZ - niceRange.zBegin) / (niceRange.zEnd - niceRange.zBegin) };
        glm::mat4 fitMat(1.0f);
        fitMat = glm::translate(fitMat, glm::vec3(translateY, translateZ, translateX));
        fitMat = glm::scale(fitMat, glm::vec3(scaleY, scaleZ, scaleX));
        fitMat = glm::translate(fitMat,  glm::vec3(centerY, -centerZ, centerX));
        fitSurfaceMatrix = fitMat;
    }
};

#endif


#endif