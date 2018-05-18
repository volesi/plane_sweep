#include "../include/plane_sweep/project.h"
#include <iostream>

int main()
{
  try
  {
    Project lab;
    lab.run();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Caught exception:\n"
              << e.what() << "\n";
  }
  catch (...)
  {
    std::cerr << "Caught unknown exception\n";
  }

  return EXIT_SUCCESS;
}
