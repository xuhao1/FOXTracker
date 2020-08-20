# Introduction
![screenshot](./docs/screenshot.png)
This is a faical head tracker for gaming usage. Perform as TrackIR or [opentrack](https://github.com/opentrack/opentrack) (pointtracker) as track camera control.

Download at [Release](https://github.com/xuhao1/FlightAgentX/releases)

opentrack is recommend to install now.
## Usage

This program is still under develop, not stable yet!

This program supports control games directly or use opentrack as backend. For now the spline functions is in develop, so use opentrack is a good idea.

Just turn your opentrack input to UDP and open this program. Everything works fine.

![c1](./docs/opentracker_config.PNG)
![c2](./docs/opentracker_config2.PNG)

Or using dcs.ini inside release folder.

Video can be found on https://www.bilibili.com/video/BV1rE411c7Wx/
## Future Plan (Maybe in a year)
Try to reinforce the robust of the tracker.
Will add a better pointtracker frontend.
Will add spline function.
Will fuse IMU with face tracker.

## LICENSE
MIT. For commerical use, you must let me know.



## 介绍
这是一个基于面部识别的头瞄，用于模拟类游戏。和TrackIR或者[opentrack](https://github.com/opentrack/opentrack)（如国内流行的pointtracker）功能类似，但是只需要摄像头。
在 [Release](https://github.com/xuhao1/FlightAgentX/releases) 下载
请事先安装opentrack

## 使用
首先这玩意还没有成熟，问题很多。体验党慎用。

你可以使用本程序直接控制游戏，或者使用opentrack作为后端。考虑到目前曲线功能还没有开发完成，使用opentrack作为后端是一个好主意。

把你的opentrack的input设置为UDP，然后一切都会工作的很美丽。
![c1](./docs/opentracker_config.PNG)
![c2](./docs/opentracker_config2.PNG)

或者使用release中附带的 dcs.ini文件。

视频见b站 https://www.bilibili.com/video/BV1rE411c7Wx/

## 未来一年内的开发计划
加强脸瞄的稳定性
加入更好的pointtracker前端
加入曲线编辑功能
把IMU融合到脸瞄中

## 协议

MIT协议。但是想商用得先告知作者，否则是侵权行为。