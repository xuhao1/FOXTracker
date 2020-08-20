path=C:\Qt\5.14.1\msvc2017\bin
mkdir Release
mkdir Release\\assets
copy ..\\build-FlightAgentX-Desktop_Qt_5_14_1_MSVC2017_32bit-Release\\release\\FlightAgentX.exe .\\Release
copy  .\\assets .\\Release\assets
cd Release
windeployqt FlightAgentX.exe