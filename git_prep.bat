@echo off
echo ==============================================================
echo [CppPlot] Git Pre-commit Cleanup and Setup
echo ==============================================================
echo.

echo [1/3] Cleaning up build artifacts...
if exist "build" (
    rmdir /s /q "build"
    echo   - Removed build/ directory
)

echo [2/3] Cleaning up generated images and temporary files...
del /s /q /f "*.exe" >nul 2>&1
del /s /q /f "*.obj" >nul 2>&1
del /s /q /f "*.pdb" >nul 2>&1
del /s /q /f "temp_plot.svg" >nul 2>&1
del /s /q /f "*.tmp" >nul 2>&1

:: Xóa sạch các ảnh sinh ra trong thư mục examples/output
if exist "examples\output" (
    del /s /q /f "examples\output\*.svg" >nul 2>&1
    del /s /q /f "examples\output\*.png" >nul 2>&1
    echo   - Cleaned examples/output/
)

echo [3/3] Checking .gitignore...
if not exist ".gitignore" (
    echo   - .gitignore missing! Please create one.
) else (
    echo   - .gitignore is present.
)

echo.
echo ==============================================================
echo Cleanup complete! You can now safely run:
echo    git add .
echo    git commit -m "Your message"
echo ==============================================================
pause
