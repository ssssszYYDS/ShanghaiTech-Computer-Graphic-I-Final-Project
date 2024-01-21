cd ./Coding

mkdir build
cd build
cmake ..
cmake --build . --config Release -j 4

cd ./Release

main.exe

cd ../../../