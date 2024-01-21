# Final Project: Fast Simulation of Mass-Spring Systems

Name: 夏康杰

Student ID: 2021533071

## Introduction

This is a GUI program using OpenGL for real-time cloth simulation. Please read the following instructions carefully before working on this project.

## Shader path

Before compiling and running the program, you can change the shader path in `src/main.cpp`, about line 77.

If you keep it unchanged, you should set your working directory to the `Coding` folder, or the program will fail finding the shader files.

## Compilation

To compile the program, set your working directory to the `Coding` folder, then

```bat
mkdir build
cd build
cmake ..
cmake --build . --config Release -j 4
```

After running this you only need to execute the last command to compile again.

If you are windows, you can just run the file: simulate.bat.

If you have any trouble compiling, please contact me by the email: xiakj@shanghaitech.edu.cn

You should be able to see a window with some contents in it if you run the compiled program.

## Controls & Settings

Running the program starts the simulation. Press esc key to quit the program.

A first person camera is provided.

- W, A, S, D, Left Control, Space: Moving
- Arrows: Look around

The simulation parameters can be tuned in `src/main.cpp`, about line 80.

## Extra

If you are not satisfied with the code skeleton, you are encouraged to modify it to achieve more interesting effects.

For example, you may modify the fragment shader for better lighting setup, or modify the definition of `RectClothSimulator` for extra functionalities.

## Statement

Refer to [course page](https://faculty.sist.shanghaitech.edu.cn/faculty/liuxp/course/cs171/).
