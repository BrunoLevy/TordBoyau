MAIN=soc.v
mkdir -p obj_dir
(cd obj_dir;  rm -f *.cpp *.o *.a VSOC; cp ../sim_main.cpp .; make -f VSOC.mk)
verilator -DBENCH -DPASSTHROUGH_PLL -Wno-fatal \
	  --top-module SOC -cc -exe sim_main.cpp $MAIN
(cd obj_dir; make -f VSOC.mk)
obj_dir/VSOC $2

