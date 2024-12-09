#ifndef PLOTSCENE_HPP
#define PLOTSCENE_HPP

#include <chrono>
#include <glm/glm.hpp>
#include "camera.hpp"
#include "shader.hpp"
#include "plot3D.hpp"

inline double GetTimeAsDouble() {
    using namespace std::chrono;
    using SecondsFP = std::chrono::duration<double>;
    return duration_cast<SecondsFP>(high_resolution_clock::now().time_since_epoch()).count();
}

class PlotScene {
public: 
    PlotScene();
    void plot(const Surface3D& surface, 
              ColormapFunc colormapFunc = viridisColormap, 
              CoordsColoringRule coloringRule = defaultColoringRule);
    void setPerspective(float fovY, float aspect, float nearPlane, float farPlane);
    void render();
    Camera camera;
//private:
    Shader surfaceShader;
    Shader textShader;
    Shader gridShader;
    Plot3D plot3d;
};

#endif