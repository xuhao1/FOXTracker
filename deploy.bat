path=C:\Qt\5.14.2\msvc2017_64\bin
mkdir Release
mkdir Release\\assets
copy ..\\build-FlightAgentX-Desktop_Qt_5_14_2_MSVC2017_64bit-Release\\release\\FlightAgentX.exe .\\Release
copy  .\\assets .\\Release\assets
cd Release
windeployqt FlightAgentX.exe
