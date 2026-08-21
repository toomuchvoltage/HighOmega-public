@echo off

REM 
REM Copyright (c) 2026 TooMuchVoltage Software Inc.
REM 
REM Permission is hereby granted, free of charge, to any person obtaining a copy
REM of this software and associated documentation files (the "Software"), to deal
REM in the Software without restriction, including without limitation the rights
REM to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
REM copies of the Software, and to permit persons to whom the Software is
REM furnished to do so, subject to the following conditions:
REM 
REM The above copyright notice and this permission notice shall be included in all
REM copies or substantial portions of the Software.
REM 
REM THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
REM IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
REM FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
REM AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
REM LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
REM OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
REM SOFTWARE.
REM 

setlocal ENABLEDELAYEDEXPANSION
cd ..
set "highomegahome=%cd%"
if "%1"=="redo" (
	del /s /q /f *.ktx
)
cd assets\models\mountains
call :recursiveCrunch
cd ..\pines
call :recursiveCrunch
del *{nomip}.ktx
cd ..\..\maps
call :recursiveCrunch
cd ..\..
set "remove_source_mat="
if not "%2"=="" (
	if "%2"=="publish" (
		set remove_source_mat=1
	)
) else (
	if "%1"=="publish" (
		set remove_source_mat=1
	)
)
if defined remove_source_mat (
	call :keepOnlyKTX
	forfiles /S /M *.blend /C "cmd /c del @file"
	forfiles /S /M *.zip /C "cmd /c del @file"
	forfiles /S /M *.fbx /C "cmd /c del @file"
	forfiles /S /M *.psd /C "cmd /c del @file"
	forfiles /S /M *.obj /C "cmd /c del @file"
	forfiles /S /M *.mtl /C "cmd /c del @file"
)
goto :eof

:recursiveCrunch
for %%f in (*.tga) do (
	set checkKtxOrigName=%%f
	set checkKtxFileName=!checkKtxOrigName:.tga=!
	echo %%f|findstr /i /L "{nocompress}">nul
	if "!ERRORLEVEL!"=="1" (
		if not exist !checkKtxFileName!.ktx (
			echo %%f|findstr /i /L "{terrain}">nul
			if "!ERRORLEVEL!"=="1" (
				echo %%f|findstr /i /L ".hgt.">nul
				if "!ERRORLEVEL!"=="0" (
					echo Greyscale: %%f
					%highomegahome%\source_material\exporter\magick.exe convert %%f %%f.png
					%highomegahome%\source_material\exporter\toktx.exe --uastc 0 --genmipmap --zcmp --target_type R --assign_oetf linear --layers 1 %%f.ktx %%f.png
					del %%f.png
					set origString=%%f
					set newString=!origString:.tga=!
					ren !origString!.ktx !newString!.ktx
				) else (
					echo %%f|findstr /i /L ".nrm.">nul
					if "!ERRORLEVEL!"=="0" (
						echo Normal map: %%f
						%highomegahome%\source_material\exporter\magick.exe convert %%f -define png:color-type=6 %%f.png
						%highomegahome%\source_material\exporter\toktx.exe --uastc 0 --genmipmap --zcmp --target_type RGB --assign_oetf linear --layers 1 %%f.ktx %%f.png
						del %%f.png
						set origString=%%f
						set newString=!origString:.tga=!
						ren !origString!.ktx !newString!.ktx
					) else (
						echo %%f|findstr /i /L ".rgh.">nul
						if "!ERRORLEVEL!"=="0" (
							echo Roughness/specularity map: %%f
							%highomegahome%\source_material\exporter\magick.exe convert %%f -define png:color-type=6 %%f.png
							%highomegahome%\source_material\exporter\toktx.exe --uastc 0 --genmipmap --zcmp --target_type RGBA --assign_oetf linear --layers 1 %%f.ktx %%f.png
							del %%f.png
							set origString=%%f
							set newString=!origString:.tga=!
							ren !origString!.ktx !newString!.ktx
						) else (
							echo Color with alpha: %%f
							%highomegahome%\source_material\exporter\magick.exe convert %%f -define png:color-type=6 %%f.png
							%highomegahome%\source_material\exporter\toktx.exe --uastc 0 --genmipmap --zcmp --target_type RGBA --assign_oetf srgb --layers 1 %%f.ktx %%f.png
							del %%f.png
							set origString=%%f
							set newString=!origString:.tga=!
							ren !origString!.ktx !newString!.ktx
						)
					)
				)
			) else (
				set cropType=1x4@
				echo %%f|findstr /i /L ".hgt.">nul
				if "!ERRORLEVEL!"=="0" (
					set cropType=1x4@
				)
				echo %%f|findstr /i /L ".nrm.">nul
				if "!ERRORLEVEL!"=="0" (
					set cropType=1x3@
				)
				echo %%f|findstr /i /L ".rgh.">nul
				if "!ERRORLEVEL!"=="0" (
					set cropType=1x3@
				)
				echo %%f|findstr /i /L ".spc.">nul
				if "!ERRORLEVEL!"=="0" (
					set cropType=1x3@
				)
				echo Terrain textures: %%f
				%highomegahome%\source_material\exporter\magick.exe convert -define png:color-type=6 -crop !cropType! +repage %%f %%f.png
				dir /b *-?.png > list.txt
				(type list.txt | find /c /v "") > linecount.txt
				set /p linecount=<linecount.txt
				echo !linecount! number of layers
				%highomegahome%\source_material\exporter\toktx.exe --uastc 0 --genmipmap --zcmp --target_type RGBA --convert_oetf srgb --layers !linecount! %%f.ktx @list.txt
				del list.txt
				del linecount.txt
				del *-?.png
				set origString=%%f
				set newString=!origString:.tga=!
				ren !origString!.ktx !newString!.ktx
			)
		) else (
			echo %%f is already converted
		)
	) else (
		echo %%f should not be compressed
	)
)
for /D %%d in (*) do (
    cd %%d
    call :recursiveCrunch
    cd ..
)
exit /b

:keepOnlyKTX
for %%f in (*.ktx) do (
	FOR /F %%A in ("%%f") DO SET fileSizeBefore=%%~zA
	set origString=%%f
	set modifiedString=!origString:.ktx=!
	FOR /F %%A in ("!modifiedString!.tga") DO SET fileSizeAfter=%%~zA
	if exist !origString! (
		if exist "!modifiedString!.tga" (
			del "!modifiedString!.tga"
		)
	)
)
for /D %%d in (*) do (
    cd %%d
    call :keepOnlyKTX
    cd ..
)
exit /b
