# Introduction
![screenshot](./docs/screenshot.png)
FlightAgentX is a facial head tracker for gaming usage. Perform as TrackIR or [Opentrack](https://github.com/opentrack/opentrack) (pointtracker) as track camera controller for Flight Simulation Games like DCS.

## Prerequst
A normal web camera.

[opentrack](https://github.com/opentrack/opentrack) is recommend to install now.
## Usage
This program is still under development, not stable yet. **I will never collect any user data from your camera.**

Download FlightAgentX at [Release](https://github.com/xuhao1/FlightAgentX/releases)


This program supports control games directly or uses Opentrack as backend. For now, the spline function is in development, so use Opentrack is a good idea. If you are using this program individually, please modify config or the config.yaml.

![c1](./docs/config.PNG)

Just turn your opentrack input to UDP and open FlightAgentX.exe. Everything works fine.
![c1](./docs/opentracker_config.PNG)
![c2](./docs/opentracker_config2.PNG)

Also, you may use [dcs.ini](./docs/dcs.ini) here.

Video can be found on https://www.bilibili.com/video/BV1ey4y1C7Za
## Future Plan (Maybe in a year)
1. Try to reinforce the robust of the tracker.
2. Will add a better pointtracker frontend.
3. Will add spline function.

## LICENSE
MIT.

## Third-party Libraries
[OpenCV](https://opencv.org/)

[dlib](http://dlib.net/)

[UGlobalHotkey](https://github.com/falceeffect/UGlobalHotkey)

[yaml-cpp](https://github.com/jbeder/yaml-cpp)

[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

[ONNX-Runtime](https://github.com/microsoft/onnxruntime)

[FSA-Net](https://github.com/shamangary/FSA-Net) LICENSE: Apache 2.0 https://github.com/shamangary/FSA-Net/blob/master/LICENSE

[OpenSeeFace](https://github.com/emilianavt/OpenSeeFace) Thanks @emilianavt 's network!

[aitracker](https://github.com/AIRLegend/aitrack)

## 介绍
这是一个基于面部识别的头瞄，用于模拟类游戏。和TrackIR或者[opentrack](https://github.com/opentrack/opentrack)（如国内流行的pointtracker）功能类似，但是只需要摄像头。
请参阅[中文文档](./docs/user_manual_chinese.md)


视频见b站 https://www.bilibili.com/video/BV1ey4y1C7Za

## 未来一年内的开发计划

1. 加强脸瞄的稳定性
2. 加入更好的pointtracker前端
3. 加入曲线编辑功能


## 协议

MIT
