@echo off
rem 갤럭시북(윈도우)용: scripts\run_all.py 실행. 인자 없이 더블클릭하면 --demo
rem 예) scripts\run_all_galaxybook.bat --target 300 --duty 70
chcp 65001 >nul
set PYTHONUTF8=1
cd /d "%~dp0.."
set ARGS=%*
if "%ARGS%"=="" set ARGS=--demo
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 scripts\run_all.py %ARGS%
) else (
    python scripts\run_all.py %ARGS%
)
pause
