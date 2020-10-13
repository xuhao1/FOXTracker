path=C:\Qt\5.14.2\msvc2017\bin
mkdir Release.x86
mkdir Release.x86\\assets
copy ..\\build-FlightAgentX-Desktop_Qt_5_14_2_MSVC2017_32bit-Release\\release\\FlightAgentX.exe .\\Release.x86
copy  .\\assets .\\Release.x86\assets
copy  .\\config.yaml .\\Release.x86\config.yaml

cd Release.x86
windeployqt FlightAgentX.exe
