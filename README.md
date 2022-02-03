# HARU
Hardware Accelerated Read Until R9 branch


# Quick start

## Getting HARU:
```sh
git clone https://github.com/beebdev/HARU -b r9_slow5 && cd HARU
```

## Setting up dependancies to run the updated RUscripts
```sh
cd RUscripts
python3 -m venv env
source env/bin/activate
pip3 install --upgrade pip
pip3 install numpy==1.18.0 pyslow5 biopython==1.69 scikit-learn==0.20.0 scipy==1.4.0 six==1.16.0 Cython
python3 setup.py install
```

## Running offline RUscripts
To run the software-based updated offline RUscripts on the example Covid-19 dataset:
```sh
cd RUscripts
python3 OfflineReadUntil.py -f dataset/fasta/nCoV-2019.reference.fasta -t MN908947.3:10000-15000 -p 4 -m models/r9.4_450bps.nucleotide.6mer.template.model -w dataset/ncov-testset/slow5 -o RUgOUT -L 3000
```

