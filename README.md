# mapora

## Dependencies

```
sudo apt install libgeographic-dev
sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/

# Folly
https://github.com/facebook/folly#ubuntu-1604-lts

## !!Important!!
## Install both fmt and folly with following:
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..

```