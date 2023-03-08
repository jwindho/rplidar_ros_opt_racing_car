# Performance documentation
The default code shows that the lidar transfers its data with a bandwidth of 35.01 KB/s and a frequency of 11.84 Hz.
After some initial changes, such as adjusting the angle from 360° to 30° (specified here in radians),
we can see that the bandwidth has decreased to 18.26 KB/s and the frequency has increased to 12 Hz.
Normally, a higher bandwidth means higher accuracy and faster response time, but since we have minimized the necessary measuring radius,
less computing power is needed to transmit the data. The increased frequency is an indication of improved performance for now.
Another indicator of improved performance is the scan_time, which has been reduced from ~0.08742s to ~0.07997s.
Further changes to the code will follow.

# Default

![benchmarkin_default](https://user-images.githubusercontent.com/84909827/223700445-5bda0c12-77ad-4d9f-90c9-4611ccebeaf3.PNG)


# Changed

![benchmarking_v1](https://user-images.githubusercontent.com/84909827/223700629-387e3d76-2d24-4b9e-abc5-66c682005718.PNG)
