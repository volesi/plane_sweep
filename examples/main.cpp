#include "../include/plane_sweep/lab_1_2.h"
#include <iostream>

int main()
{
  try
  {
    Lab_1_2 lab;
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
