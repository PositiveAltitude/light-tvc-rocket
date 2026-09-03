# Light Robot Core v1.0 software
This is a prototype of the software for "light-robot-core" board. Work in progress.


### Project structure
#### light-robot-core-embedded
A ESP32-IDF based software for ESP32-C3 microcontroller. Has a lot of requirements, check out ESP-IDF documentation to learn what you need to install.
#### light-robot-core-frontend
A control panel for light-robot-core. Built with WASM and YEW framework. You can use `trunk serve` to take a look at that locally with Trunk packager.
`light-robot-core-embedded` indirectly depends on that. There is a build script that packages frontend with Trunk, then GZIPs all files and then these files are embedded directly into firmware binaries. 
#### light-robot-core-api
Schema objects for communications between frontend and the board.
#### light-robot-core-simulation
WIP