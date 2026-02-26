#include "imgui_harness.h"
#include "imgui.h"

// Function to initialize ImGui
void InitImGui() {
    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    
    // Setup ImGui style
    ImGui::StyleColorsDark();
}

// Function to render ImGui frame
void RenderImGuiFrame() {
    ImGui::NewFrame();
    
    // Example ImGui window
    ImGui::Begin("DOPE ImGui Harness");
    ImGui::Text("Welcome to the DOPE ImGui Harness!");
    ImGui::End();
    
    ImGui::Render();
}

// Function to cleanup ImGui
void CleanupImGui() {
    ImGui::DestroyContext();
}