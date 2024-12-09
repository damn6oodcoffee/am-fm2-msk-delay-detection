#ifndef TEXT_RENDER_HPP
#define TEXT_RENDER_HPP

#include <unordered_map>
#include <string>
#include <glad/glad.h>
//#include <gl/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <ft2build.h>
#include FT_FREETYPE_H


struct Character {
    unsigned int TextureID; // ID handle of the glyph texture
    glm::ivec2   Size;      // Size of glyph
    glm::ivec2   Bearing;   // Offset from baseline to left/top of glyph
    unsigned int Advance;   // Horizontal offset to advance to next glyph
};

struct BitmapInfo {
    FT_Bitmap bitmap;
    FT_Int bitmap_left;
    FT_Int bitmap_top;
    FT_Vector advance;
};

class FreetypeWrapper {
public:
    FreetypeWrapper(const std::string& fontName) 
        : ft{}, face{}
    {
        if (FT_Init_FreeType(&ft))
            throw std::runtime_error("Failed to init FreeType Library");
        if (FT_New_Face(ft, fontName.c_str(), 0, &face)) 
            throw std::runtime_error("Failed to load font");
        FT_Set_Pixel_Sizes(face, 0, 48);
    }

    ~FreetypeWrapper() {
        FT_Done_Face(face);
        FT_Done_FreeType(ft);
    }

    BitmapInfo loadChar(unsigned long charCode) {
        if (FT_Load_Char(face, charCode, FT_LOAD_RENDER))
            throw std::runtime_error("Failed to load glyph");
        BitmapInfo bmpInfo { 
            face->glyph->bitmap, 
            face->glyph->bitmap_left, 
            face->glyph->bitmap_top,
            face->glyph->advance 
        };
        return bmpInfo;
    }

private:
    FT_Library ft;
    FT_Face face;
};


class TextRenderer {
public:
    TextRenderer(const std::string& fontName)
        : characters{}
    {
        loadChars(fontName);
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
    }

    ~TextRenderer() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);

    }

    // lest you get memory leaks, this shouldn't be called more than once
    void bind() {
        if (!firstTextBind)
            return;
        firstTextBind = false;
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
        glBindVertexArray(0);
    }

    void setFont(const std::string& fontName) {
        loadChars(fontName);
    }
    
    void render(std::string text, float x, float y, float scale) {
        glActiveTexture(GL_TEXTURE0);
        glBindVertexArray(VAO);
        //float xpos { 0.0f }, ypos { 0.0f };
        float xpos { -(stringPixelWidth(text) * scale) / 2 }, ypos { 0.0f };
        for (auto&& c : text) {
            auto&& ch { characters[c] };
            xpos += ch.Bearing.x * scale;
            ypos = (ch.Bearing.y - ch.Size.y) * scale;

            float w { ch.Size.x * scale };
            float h { ch.Size.y * scale };
            float vertices[6][4] {
                {     xpos, h + ypos, 0.0f, 0.0f },
                {     xpos,     ypos, 0.0f, 1.0f },
                { w + xpos,     ypos, 1.0f, 1.0f },

                {     xpos, h + ypos, 0.0f, 0.0f },
                { w + xpos,     ypos, 1.0f, 1.0f },
                { w + xpos, h + ypos, 1.0f, 0.0f }
            };

            glBindTexture(GL_TEXTURE_2D, ch.TextureID);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
            glDrawArrays(GL_TRIANGLES, 0, 6);
            xpos += (ch.Advance >> 6) * scale; // bitshift by 6 to get value in pixels (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
        }
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }


private:
    std::unordered_map<unsigned char, Character> characters;
    unsigned int VAO, VBO;
    bool firstTextBind = true;

    void loadChars(const std::string fontName) {
        FreetypeWrapper ftWrapper(fontName);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        for (unsigned char c { 0 }; c < 128; ++c) {
            BitmapInfo bmpInfo = ftWrapper.loadChar(c);
            unsigned int texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(
                GL_TEXTURE_2D,
                0,
                GL_RED,
                bmpInfo.bitmap.width,
                bmpInfo.bitmap.rows,
                0,
                GL_RED,
                GL_UNSIGNED_BYTE,
                bmpInfo.bitmap.buffer
            );
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            Character character {
                texture,
                glm::ivec2(bmpInfo.bitmap.width, bmpInfo.bitmap.rows),
                glm::ivec2(bmpInfo.bitmap_left, bmpInfo.bitmap_top),
                static_cast<unsigned int>(bmpInfo.advance.x)
            };
            characters.insert({c, character});
        }
        glBindTexture(GL_TEXTURE_2D, 0);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    }

    int stringPixelWidth(const std::string& text) {
        int width { 0 };
        for (auto&& c: text) {
            auto&& ch { characters[c] };
            width += ch.Bearing.x + (ch.Advance >> 6);
        }
        return width;
    }
};

#endif