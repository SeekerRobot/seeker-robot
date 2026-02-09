# libs_external

Used in the weird case that we can't include things in the lib common folder, but also probably shouldn't be including it in the src folder.

Having microros libraries for the ESP32 stored in here guarnatees that all targets involving the ESP32 will only have to build microros once.