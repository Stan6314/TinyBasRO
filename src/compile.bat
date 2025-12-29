@ECHO OFF
REM %1 = Path to file

CD %1

IF EXIST error.log DEL error.log

REM Compile

make >> error.log 2>&1



