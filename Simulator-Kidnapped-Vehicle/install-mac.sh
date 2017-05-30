brew install openssl libuv cmake
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig
cd build
# This is to make the shell script work on a Mac
cmake -DOPENSSL_ROOT_DIR=/usr/local/Cellar/openssl/1.0.2k -DOPENSSL_LIBRARIES=/usr/local/Cellar/openssl/1.0.2k/lib ..
make
make
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
