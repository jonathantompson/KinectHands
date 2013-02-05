//
//  main.cpp
//
//  Created by Jonathan Tompson on 5/15/12.
// 

#include <stdexcept>
#include "app/app.h"
#include "windowing/window_base.h"

using app::App;

int main(int argc, char *argv[]) {
  try {
    App::InitApp(argc, argv);
    App::RunApp();
  } catch(std::runtime_error e) {
    printf("std::runtime_error caught!:\n");
    printf("  %s\n", e.what());
    // Also open a window box (since this is a .app or .exe and we wont have
    // access to the terminal for error messages outside of XCode/VS)
    windowing::ErrorBox(e.what());
  }
  App::KillApp();
}
