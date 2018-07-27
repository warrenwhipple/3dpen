# 3D Pen

An experiment in dead reckoning using iPhone accelerometer and gyroscope sensors.

![Animated gif demo](https://media.giphy.com/media/39qD4sMKrpBVLYJRC7/giphy.gif)

Apple added gyroscopic MEMS accelerometers to the iPhone 4 in 2010, to compliment the lateral MEMS accelerometers introduced in the iPhone 1. Hypothetically, it should be possible to track an iPhone’s position in space by integrating over the acceleration data to calculate lateral and rotational velocity, and integrating again to calculate lateral and rotational position.

This project is my attempt to apply this idea to use an iPhone as pseudo pen that can draw in 3D space and view the results on screen.

Subsequent googling revealed the idea has a long history.

- [Wikipedia: Dead reckoning](https://en.wikipedia.org/wiki/Dead_reckoning)
- [A comparison of Pedestrian Dead-Reckoning algorithms using a low-cost MEMS IMU (2009)](https://ieeexplore.ieee.org/document/5286542/)
- [Sensor Fusion on Android Devices: A Revolution in Motion Processing](https://www.youtube.com/watch?v=C7JQ7Rpwn2k)

While this 3D Pen app does work in short bursts for drawing a second or two at a time, I was unable to address the drift amplification problems, and abandoned the project. The idea could likely be improved with sensor fusion algorithms + Kalman filtering, and/or combining the accelerometer data with camera data a la Apple’s later introduced ARKit. Though I have not looked into these approaches recently.
