# EMATM0054 UoB: Robotics Systems Assessment 1 (Search & Rescue Challenge)

## Introduction

This project uses Platform.io tools for programming. The demonstration is [here](https://youtu.be/bpetnIimU6M)

### Assessment 1 Self-Assessment

#### Demonstration 1: 
- **Search** (Successful, stopped after 2 minutes [**55 Marks**]) 
- **Return to Start**: (Stops with any part of the robot partially over the grey-dashed line. [**7 Marks**])

#### Demonstration 2:
*Dice Roll (X, Y): (1, 3)*
- **Searched and stopped** (2 minutes minimum, or when magnet found) [**55 Marks**]
- **Locate Magnet** (Magnet Found, Robot Stopped [**10 Marks**])
- **Return to Start** (stops with any part of the robot partially over the grey-dashed line (partial) [**7 Marks**])
- **“Return to Magnet”** (stops completely covering the silver disk of the magnet (complete) [**24 Marks**])

#### Demonstration 3:
*Dice Roll (X, Y): (3, 1)*
- **Searched and stopped** (2 minutes minimum, or when magnet found) [**55 Marks**]
- **Locate Magnet** (Magnet Found, Robot Stopped [**10 Marks**])
- **Return to Start** (stops with any part of the robot partially over the grey-dashed line (partial) [**7 Marks**])
- **“Return to Magnet”** (stops completely covering the silver disk of the magnet (complete) [**24 Marks**])
  

## Project Structure

The project have the following structure:

```
my_project/
├── .pio/
├── include/
├── lib/
├── src/
│   └── main.cpp
├── test/
└── platformio.ini
```

- `src/`: This is where your main code goes.
- `lib/`: For project-specific libraries.
- `include/`: For project-specific header files.
- `platformio.ini`: The configuration file for your project.

You may want to change `upload_port` when building and uploading your code inside `platformio.ini` 

This project depends on 
1. `Arduino` for Arduino functions
2. `pololu/PololuOLED@^2.0.0` for OLED display
3. `pololu/LIS3MDL@^2.0.0` for magnetometer

## Entry point

`src/main.cpp`.


## Building and Uploading

1. Build the project:
```sh
pio run
```

2. Upload the code to your board:
```sh
pio run --target upload
```