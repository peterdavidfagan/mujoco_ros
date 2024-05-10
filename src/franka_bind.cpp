#include <pybind11/pybind11.h>

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(franka_env, m)
{
    m.doc() = R"(
            Python bindings for moveit_ros environments for the purpose of supporting the python interactive viewer.
            )";

  // Provide custom function signatures
  py::options options;
  options.disable_function_signatures();

  // construct a python class for franka ros simulation instance  

  // pass model and data instances from franka ros simulation for use by interactive viewer
  
  // test a basic function
  m.def("add", &add, R"(
        Add two numbers

        Some other explanation about the add function.
    )");
}