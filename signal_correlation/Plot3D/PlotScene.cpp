
#include "PlotScene.hpp"


PlotScene::PlotScene()
    : camera(glm::vec3(0.0f, 0.0f, 3.0f))
    , surfaceShader("shaders/surface.vert", "shaders/surface.frag")
    , textShader("shaders/text.vert", "shaders/text.frag")
    , gridShader("shaders/grid.vert", "shaders/grid.frag")
    , plot3d("font/IBMPlexSans-Regular.ttf")
{}

void PlotScene::plot(const Surface3D& surface, 
              ColormapFunc colormapFunc, 
              CoordsColoringRule coloringRule)
{
    plot3d.plot(surface, colormapFunc, coloringRule);
}

void PlotScene::setPerspective(float fovY, float aspect, float nearPlane, float farPlane) {
    glm::mat4 projection = glm::perspective(fovY, aspect, nearPlane, farPlane);
    surfaceShader.use();
    surfaceShader.setMat4("projection", glm::value_ptr(projection));
    textShader.use();
    textShader.setMat4("projection", glm::value_ptr(projection));
    gridShader.use();
    gridShader.setMat4("projection", glm::value_ptr(projection));
}

void PlotScene::render() {

    glm::mat4 view = camera.GetViewMatrix();

    textShader.use();
    textShader.setMat4("view", glm::value_ptr(view));
    textShader.setVec3("textColor", glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));

    gridShader.use();
    gridShader.setMat4("view", glm::value_ptr(view));

    surfaceShader.use();
    surfaceShader.setMat4("view", glm::value_ptr(view));
    plot3d.render(gridShader, textShader, surfaceShader);
}