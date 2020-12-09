echo "Making all versions"
echo "Please ignore all warnings"
cd CPU-N-Squared/
echo "Making CPU N^2 version"
make
cd ../CPU-Coarse-Grid/
echo "Making CPU Coarse Grid version"
make
cd ../GPU-N-Squared/
echo "Making GPU N^2 version"
sh make_gpu.sh
cd ../GPU-Coarse-Grid/
echo "Making GPU Coarse Grid version"
sh make_gpu.sh
cd ..
echo "Make complete"
