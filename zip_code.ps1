# zip all code in current folder and save it to zf-code.zip
# accepted suffix: .txt, .cpp, .h,
# no subfolder, no file in subfolder

# create a temp folder
New-Item -ItemType Directory -Path .\temp -Force

# copy files that matched the rules above to a temp folder
Select-String -Path .\CMakeLists.txt -Pattern ".*" | Select-Object -ExpandProperty Path | Copy-Item -Destination .\temp -Force
Select-String -Path .\*.cpp -Pattern ".*" | Select-Object -ExpandProperty Path | Copy-Item -Destination .\temp -Force
Select-String -Path .\*.h -Pattern ".*" | Select-Object -ExpandProperty Path | Copy-Item -Destination .\temp -Force

# zip the temp folder
Compress-Archive -Path .\temp\* -DestinationPath .\code.zip -Force

# delete the temp folder
Remove-Item -Path .\temp -Recurse
