#pragma once

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>

#include <GL/gl3w.h>            // Initialize with gl3wInit()
#include <GLFW/glfw3.h>

namespace TS_SfM {
class Viewer {
  public:
    Viewer(){};
    ~Viewer(){};

    int Run();
  private:

};
};
