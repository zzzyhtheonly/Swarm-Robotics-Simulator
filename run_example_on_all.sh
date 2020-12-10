cd CPU-N-Squared/
echo "Executing CPU-N-Squared/simulator -i 1000 20 5000 2 20 300"
./simulator -i 1000 20 5000 2 20 300
mv log.txt log2.txt
echo "CPU N^2 complete"
cd ../CPU-Coarse-Grid/
echo "Executing CPU-Coarse-Grid/simulator -i 1000 20 5000 2 20 300"
./simulator -i 1000 20 5000 2 20 300
mv log.txt log2.txt
echo "CPU Coarse Grid complete"
cd ../GPU-N-Squared/
echo "Executing GPU-N-Squared/simulator -i 1000 20 5000 2 20 300"
./simulator -i 1000 20 5000 2 20 300 > log2.txt
mv log.txt stdout.txt
echo "GPU N^2 complete, stdout written to 'stdout.txt'"
cd ../GPU-Coarse-Grid/
echo "Executing GPU-Coarse-Grid/simulator -i 1000 20 5000 2 20 300"
./simulator -i 1000 20 5000 2 20 300
mv log.txt log2.txt
echo "GPU Coarse Grid complete"
echo "All complete"
