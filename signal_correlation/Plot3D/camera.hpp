#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <algorithm>
//#include <glad/glad.h>
#include <glad/glad.h>
//#include <gl/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

enum class CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
};

const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 15.0f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

class Camera {
public:
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    float Yaw;
    float Pitch;
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;
    
    glm::vec3 pos;
    glm::vec3 center;
    glm::vec3 up;
    float radius;
    float azimuth;
    float elevation;

    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
           glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
           float yaw = YAW,
           float pitch = PITCH)
           : Front(glm::vec3(0.0f, 0.0f, -1.0f))
           , MovementSpeed(SPEED)
           , MouseSensitivity(SENSITIVITY)
           , Zoom(ZOOM)
           , Position(position)
           , WorldUp(up)
           , Yaw(yaw)
           , Pitch(pitch)
    {
        updateCameraVectors();
        pos = glm::vec3(2.0f, 2.0f, 2.0f);
        center = glm::vec3(0.5f, 0.5f, 0.5f);
        this->up = glm::vec3(0.0f, 1.0f, 0.0f);
        radius = 2.65f;
        azimuth = 45.0f;
        elevation = 45.0f;
    }

    glm::mat4 GetViewMatrix() {
        /* own look at function:
        glm::mat4 rotation = glm::transpose(
            glm::mat4(
                glm::vec4(Right[0], Right[1], Right[2], 0.0f),
                glm::vec4(Up[0], Up[1], Up[2], 0.0f),
                glm::vec4(-Front[0], -Front[1], -Front[2], 0.0f),
                glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
            )
        );
        glm::mat4 translation = glm::mat4(
            glm::vec4(1.0f, 0.0f, 0.0f, 0.0f),
            glm::vec4(0.0f, 1.0f, 0.0f, 0.0f),
            glm::vec4(0.0f, 0.0f, 1.0f, 0.0f),
            glm::vec4(-Position[0], -Position[1], -Position[2], 1.0f)
        );
        glm::mat4 view = rotation * translation;
        return view;
        */
        setUpPosVector();
        return glm::lookAt(pos, center, up);
        return glm::lookAt(Position, Position + Front, Up);
    }
    
    void setUpPosVector() {
        float azRads = glm::radians(azimuth);
        float elRads = glm::radians(elevation);
        float planeRadius{ radius * cos(elRads) };
        float x{ planeRadius * cos(azRads) };
        float z{ planeRadius * sin(azRads) };
        float y{ radius * sin(elRads) };
        pos = { x, y, z };
        pos += center;
        //pos = glm::vec3(2.0f, 2.0f, 2.0f);
    }

    void rotateHorizontally(float offset) {
        azimuth += offset;
    }

    void rotateVertically(float offset) {
        elevation += offset;
        elevation = std::clamp(elevation, 0.0f, 89.0f);
    }

    glm::mat4 FaceCameraDirectionMatrix() {
        glm::mat4 rotation = glm::mat4(
            glm::vec4(Right[0], Right[1], Right[2], 0.0f),
            glm::vec4(Up[0], Up[1], Up[2], 0.0f),
            glm::vec4(-Front[0], -Front[1], -Front[2], 0.0f),
            glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
        );
        return rotation;
    }
    

    void ProcessKeyboard(CameraMovement direction, float deltaTime) {
        float velocity = MovementSpeed * deltaTime;
        // Use fpsFront instead of Front for horizontal plane movement only ("FPS" camera):
        // glm::vec3 fpsFront = glm::normalize(glm::vec3(Front[0], 0.0f, Front[2]));
        if (direction == CameraMovement::FORWARD)
            Position += Front * velocity;
        if (direction == CameraMovement::BACKWARD)
            Position -= Front * velocity;
        if (direction == CameraMovement::LEFT)
            Position -= Right * velocity;
        if (direction == CameraMovement::RIGHT)
            Position += Right * velocity;
    }

    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true) {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Yaw   += xoffset;
        Pitch += yoffset;

        // make sure that when pitch is out of bounds, screen doesn't get flipped
        if (constrainPitch)
        {
            if (Pitch > 89.0f)
                Pitch = 89.0f;
            if (Pitch < -89.0f)
                Pitch = -89.0f;
        }

        // update Front, Right and Up Vectors using the updated Euler angles
        updateCameraVectors();
    }

    void ProcessMouseScroll(float yoffset) {
        Zoom -= yoffset;
        if (Zoom < 1.0f)
            Zoom = 1.0f;
        if (Zoom > 45.0f)
            Zoom = 45.0f;
    }

private:
    // calculates the front vector from the Camera's (updated) Euler Angles
    void updateCameraVectors() {
        // calculate the new Front vector
        glm::vec3 front;
        front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        front.y = sin(glm::radians(Pitch));
        front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        Front = glm::normalize(front);
        // also re-calculate the Right and Up vector
        Right = glm::normalize(glm::cross(Front, WorldUp));  // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
        Up    = glm::normalize(glm::cross(Right, Front));
    }
};

#endif