FILE_DIR=$(realpath $(dirname $0))


if [ ! -d "$FILE_DIR/../../../../externals/Pangolin" ]; then 
    echo "Cloning paho"
    cd $FILE_DIR/../../../../externals
    git clone https://github.com/stevenlovegrove/Pangolin
fi

# remove build if it exists
if [ -d "$FILE_DIR/../../../../externals/Pangolin/build" ]; then rm -rf $FILE_DIR/../../../../externals/Pangolin/build; fi

cd $FILE_DIR/../../../../externals/Pangolin/

# Install dependencies (as described above, or your preferred method)
./scripts/install_prerequisites.sh recommended

# Configure and build
cmake -B build
cmake --build build

