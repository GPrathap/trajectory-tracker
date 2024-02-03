#!/bin/bash
set -e

TMP_DIR="/tmp"
CURRENT_DIR=`pwd` 
install_osqp_eigen() {
    sudo apt-get install -y  libeigen3-dev
    echo "eigen3 is installed successfully!"

    echo "Prepare to install osqp"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include | grep osqp);then
        echo "osqp is already installed......"
    else
        cd $TEMP_DIR
        git clone --recursive https://github.com/oxfordcontrol/osqp
        cd osqp
        mkdir build && cd build
        cmake -G "Unix Makefiles" ..
        cmake --build .
        sudo cmake --build . --target install
        echo "osqp installed successfully"
    fi
    sudo ldconfig 
    echo "Prepare to install osqp-eigen"
    if ( ls /usr/local/include | grep OsqpEigen);then
        echo "OsqpEigen is already installed......"
    else
        cd $TEMP_DIR
        git clone https://github.com/robotology/osqp-eigen.git
        cd osqp-eigen
        mkdir build && cd build
        cmake ../
        make -j$(nproc) 
        sudo make install
        echo "osqp-eigen installed successfully"
    fi
    cd $TMP_DIR
    rm -rf $TEMP_DIR
    sudo ldconfig 
}

install_glog() {
    sudo apt-get install -y  libgoogle-glog-dev
    echo "glog is installed successfully!"
}

install_g2o() {
    echo "Prepare to install g2o"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include | grep g2o);then
        echo "g2o is already installed......"
    else
        cd $TEMP_DIR
        git clone https://github.com/ros2-gbp/libg2o-release.git
        cd libg2o-release
        git checkout debian/humble/libg2o
        mkdir build && cd build
        cmake ../
        make -j$(nproc) 
        sudo make install
        echo "g2o installed successfully"
    fi
    cd $TMP_DIR
    rm -rf $TEMP_DIR
    sudo ldconfig 
}


install_gflags() {
    sudo apt-get install -y  libgflags-dev 
    echo "gflags is installed successfully!"
}

install_ros_packegs_from_apt() {
    sudo apt-get install -y ros-humble-tf2*  ros-humble-grid-map ccache libopenvdb-dev 
}

install_octomap(){
    echo "Prepare to install octomap"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include | grep dynamicEDT3D);then
        echo "octomap is already installed......"
    else
        sudo apt-get install wget -y
        cd $TEMP_DIR
        wget https://github.com/OctoMap/octomap/archive/refs/tags/v1.9.6.tar.gz
        tar -xvf v1.9.6.tar.gz
        cd octomap-1.9.6/
        mkdir build && cd build
        cmake ..
        make -j$(nproc)
        sudo make install 
        echo "octomap installed successfully"
    fi
}

install_control_box_rst(){
    echo "Prepare to install control_box_rst"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include | grep control_box_rst);then
        echo "control_box_rst is already installed......"
    else
        sudo apt-get install wget -y
        cd $TEMP_DIR
        git clone https://github.com/GPrathap/control_box_rst.git
        cd control_box_rst/
        mkdir build && cd build
        cmake -DIPOPT_DIR=/usr/local ..
        make -j$(nproc)
        sudo make install 
        echo "control_box_rst installed successfully"
    fi
}



install_openvdb(){
    echo "Prepare to install opendvb"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include/ | grep openvdb);then
        echo "openvdb is already installed......"
    else
        cd $TEMP_DIR
        git clone https://github.com/AcademySoftwareFoundation/openvdb.git
        cd openvdb
        git checkout v10.0.1
        mkdir build && cd build
        cmake -DCMAKE_BUILD_TYPE=Release -DOPENVDB_BUILD_PYTHON_MODULE=OFF -DOPENVDB_CORE_SHARED=ON -DOPENVDB_CORE_STATIC=OFF -DOPENVDB_BUILD_BINARIES=OFF -DOPENVDB_CXX_STRICT=OFF -DUSE_CCACHE=ON ..
        make -j$(nproc)
        sudo make install 
        sudo ldconfig 
        # echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> ~/.bashrc
        cd $TMP_DIR
        rm -rf $TEMP_DIR
        echo "openvdb installed successfully"
        # 
    fi
}

install_ipopt_solver(){
    # echo "Prepare to install opendvb"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include/ | grep coin);then
        echo "ipopt_solver is already installed......"
    else
        sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev -y 
        cd $TEMP_DIR
        git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
        cd ThirdParty-ASL
        ./get.ASL
        ./configure
        make
        sudo make install
        cd $TEMP_DIR
        git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
        cd ThirdParty-Mumps
        ./get.Mumps
        ./configure
        make -j$(nproc)
        sudo make install
        # cd $TEMP_DIR
        # git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
        # cp $CURRENT_DIR/coinhsl-archive-2022.12.02.tar.gz $TEMP_DIR/ThirdParty-HSL/
        # cd ThirdParty-HSL
        # tar -xvf coinhsl-archive-2022.12.02.tar.gz 
        # mv coinhsl-archive-2022.12.02 coinhsl 
        # ./configure
        # make -j$(nproc)
        # sudo make install
        cd $TEMP_DIR
        git clone --single-branch --branch stable/3.13 https://github.com/coin-or/Ipopt.git 
        cd Ipopt
        mkdir -p build
        cd build 
        ../configure --prefix=/usr/local ADD_FFLAGS=-fPIC ADD_CFLAGS=-fPIC ADD_CXXFLAGS=-fPIC
        make -j$(nproc)
        sudo make install
        sudo ldconfig 
        cd /usr/local/include 
        sudo cp -r coin-or coin 
        echo "export IPOPT_DIR=/usr/local" >> ~/.bashrc
        # echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> ~/.bashrc
        cd $TMP_DIR
        rm -rf $TEMP_DIR
        echo pkg-config --libs ipopt
        echo "ipopt_solver installed successfully"
    fi
}

install_kindr(){
    echo "Prepare to install kindr"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include | grep kindr);then
        echo "kindr is already installed......"
    else
        cd $TEMP_DIR
        git clone https://github.com/GPrathap/kindr.git
        cd kindr 
        git checkout humble_dev 
        mkdir build && cd build
        cmake ..
        make -j$(nproc)
        sudo make install 
        echo "kindr installed successfully"
    fi
}

install_decomp_ros(){
    echo "Prepare to install DecompROS"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include | grep DecompROS);then
        echo "DecompROS is already installed......"
    else
        cd $TEMP_DIR
        git clone https://github.com/GPrathap/DecompROS.git
        cd DecompROS 
        git checkout humble_dev 
        mkdir build && cd build
        cmake ..
        make -j$(nproc)
        sudo make install 
        echo "DecompROS installed successfully"
    fi
}



main() {
    # sudo apt-get update
    install_glog
    install_gflags
    install_osqp_eigen
	install_ros_packegs_from_apt
    install_g2o
    install_octomap
    install_openvdb
    install_ipopt_solver
    install_control_box_rst
    install_kindr
    install_decomp_ros
}

main

