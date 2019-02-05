# Google-Vision-AIY-on-Wheels
## Motivation
After playing with Google Vision AIY kit I decided to put it on wheels and turn a cheap remote-controlled Arduino toy car kit into autonomous mobile Edge AI device. The new robot uses face-recognition model pre-installed on Google Vision AIY kit to search and approach faces or their images. Full documentation and methodology are provided in this tutorial. 
This is the demo of the final product:
<a href="https://youtu.be/e1W9Q3tUM7I" target="_blank"></a>

<a href="https://youtu.be/e1W9Q3tUM7I
" target="_blank"><img src="https://img.youtube.com/vi/e1W9Q3tUM7I/0.jpg" 
alt="Google Vision AIY Kit on Wheels" width="480" height="360" border="10" /></a>

## Warning
Please use common sense and be careful when playing with this robot - it is searching for human faces and drives toward them so don't put your face too close to the robot and don't let little children play with it without supervision!

## Hardware
- [Google AIY Vision](https://www.target.com/p/google-vision-kit-aiy/-/A-53417081)
- [UNIROI Arduino Robot Car Kit (with Arduino UNO R3 included)](https://www.amazon.com/UNIROI-Ultrasonic-Infrared-Tracking-Required/dp/B07CVS1LBT)
- 3-Pin 15cm Servo Extension Cable Male-to-Male (3 units)
- [SparkFun Jumper Wire - 0.1", 6-pin, 12"](https://www.sparkfun.com/products/10376)
- [SparkFun Logic Level Converter - Bi-Directional](https://www.sparkfun.com/products/12009)
- Wires (generic) (5 units)
- ZILU Smart Power Basic 4400mAh Portable Charger or similar (to power Google Vision AIY kit)
- [Black Nylon Screw and standoff](https://www.digikey.com/product-detail/en/adafruit-industries-llc/3299/1528-2339-ND/)
- Plywood Sheets (as mount plates) (3 units) - I bought them at JoAnn Fabrics 
- [Camera Mounting Screw Adapter](https://www.alibaba.com/product-detail/Camera-Mounting-Screw-Camera-Flash-Adapter_60263239907.html)
- [Reusable Hook & Loop Strong Grip Fastener Roll 1" x 16](https://www.amazon.com/dp/B01K3JO3QO)
- [SparkFun Break Away Headers - Straight](https://www.sparkfun.com/products/116)
- [Double Sided PCB Board](https://www.amazon.com/dp/B075VSJCD2)
- [YI Action Camera (Optional)](https://www.amazon.com/YI-Action-Camera-US-White/dp/B016EIGEGU)

## Robot Chassis
I purchased **UNIROI Smart Robot Car Kit Arduino Robot Kit** on amazon.com which comes with Arduino UNO R3 board and documentation including chassis schematics, assembly instructions and Arduino sketch *Comprehensive_Experiment.ino* uploaded to UNO R3 by a manufacturers (located is /arduino folder.)

<img src="images/chassis_with_drilled_holes.jpg">
