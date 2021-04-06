@echo off

rem Install xming git docker
rem Start xlaunch and configure 'No Access Control' to the X Server

for /f "delims=[] tokens=2" %%a in ('ping -4 -n 1 %ComputerName% ^| findstr [') do set NetworkIP=%%a
set DISPLAY=%NetworkIP%:0.0

set IMAGE=mdei/shieldone:latest

echo NetworkIP %NetworkIP%
echo Display %DISPLAY%
echo Docker Image %IMAGE%

docker pull %IMAGE%

SET HOST_PATH="%cd%"
SET DOCKER_PATH="/home/user/"

echo HOST %HOST_PATH%
echo DOCKER %DOCKER_PATH%

docker run -it --rm ^
	--volume=%HOST_PATH%:%DOCKER_PATH% ^
	--workdir=%DOCKER_PATH% ^
	-e DISPLAY=%DISPLAY% ^
	%IMAGE% ^
	bash -c "./run.sh test"
