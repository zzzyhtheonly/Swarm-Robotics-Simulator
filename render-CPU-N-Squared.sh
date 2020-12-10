cd CPU-N-Squared/
if [[ -f log2.txt ]]
then
	./simulator -l
else
	echo "log2.txt not detected, please run simulation and ensure that logging file is saved at log2.txt. Running the example script will do."
fi
cd ..
