#include <iostream>
#include "imgui_harness.h"
#include "bce/dope.h"

int main() {
    // Initialize the GUI
    if (!InitializeImGui()) {
        std::cerr << "Failed to initialize ImGui." << std::endl;
        return -1;
    }

    // Main application loop
    while (true) {
        // Start a new frame
        StartImGuiFrame();

        // Render the GUI components
        RenderGUI();

        // End the frame and render
        EndImGuiFrame();
    }

    // Cleanup and exit
    CleanupImGui();
    return 0;
}