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
I purchased **UNIROI Smart Robot Car Kit Arduino Robot Kit** on amazon.com which comes with **Arduino UNO R3** board and documentation including chassis schematics, assembly instructions and Arduino sketch *Comprehensive_Experiment.ino* uploaded to UNO R3 by a manufacturers (located in **/arduino** folder.)

<img width="800" height="600" src="images/chassis_with_drilled_holes.jpg">

## Adding Google Vision AIY kit

I drilled two holes in the front part of the chassis (marked with red circles on the image above) and used plastic standoffs and screws to attach **plywood sheets** from Joann Fabrics (colored in black) which I used as mount plates for **Google Vision AIY kit** and other components. 

The Google Vision AIY kit was attached to the upper deck with the **Camera Mounting Screw adapter**.

Lower deck was used to carry a power supply **ZILU Smart Power Basic 4400mAh Portable Charger** for Google Vision AIY kit (attached with Velcro tape the top) and the **SparkFun Logic Level Converter** (at the bottom.) Bi-Directional Logic Level Converter was used to connect pins of Google Vision AIY kit with Arduino pins (more on this in the section Connecting Google Vision AIY kit with Arduino below.)

Make sure that the charging port of the power supply is accessible so you could charge it without removing the upper deck. Also, when selecting the power supply for Google Vision AIY kit make sure it can provide at least 2.1 Amps as mentioned on Google Vision kit's website and have a capacity sufficient to run the kit for at least 20-30 minutes (capacity greater than 3000mAh should be fine.)

| Upper deck with mount screw for Google kit | Lower deck with attached power bank |
|--------------------------------------------|-------------------------------------|
| <img width="400" height="300" src="images/Mount plate upper deck.jpg"> | <img width="400" height="300" src="images/Mount plate lower deck.jpg"> | 
| Lower deck with PCB (bottom view) with PCB board with LLC) | Lower deck assembled |
|------------------------------------------------------------|----------------------| 
| <img width="400" height="300" src="images/Lower deck bottom view.jpg"> | <img width="400" height="300" src="images/Lower deck assembled.jpg"> |
