MahonyAHRS
==========

**Mahony's sensor fusion algorithm.** *Forked from [PaulStoffregen][forkSource].*

Converts raw IMU readings to Euler angles using Mahony's sensor fusion algorithm.


Resources
---------
 - [Examples][examples]
 - [API reference][doxygen classes]


Requirements
------------
 - Microcontroller and IMU device


Quick start
-----------
```c++
// Include IMU library.
#include <MahonyAHRS.hpp>

...
Mahony mahony(samplePeriod);

void setup() {}

void loop() {
  // Read IMU.
  // Convert gyrosope data to radians per second.

  float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
  if (timeToUpdate()) {
    mahony.update(yaw, pitch, roll, // The returned Euler angles
                  accelX, accelY, accelZ, // 3-axis acceleration
                  gyroX, gyroY, gyroZ); // 3-axis angular velocity
  }
  ...
}
```


License
-------
The MIT License (MIT)

Copyright (c) 2016 Vasil Kalchev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

[forkSource]: https://github.com/PaulStoffregen/MahonyAHRS
[examples]: https://github.com/VaSe7u/MahonyAHRS/tree/master/examples
[doxygen classes]: https://vase7u.github.io/MahonyAHRS/Doxygen/html/annotated.html
