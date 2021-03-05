CXXFLAGS = -I.. -g
LDFLAGS = -g
ENV = CPLUS_INCLUDE_PATH=$(PWD)

run: test_ds1603l
	./$<

test_ds1603l: test_ds1603l.o ds1603l.o
	$(CXX) $(LDFLAGS) -o $@ $^

ds1603l.o: ../ds1603l.cpp
	env $(ENV) $(CXX) $(CXXFLAGS) -c -o $@ $^

test_ds1603l.o: test_ds1603l.cpp
	env $(ENV) $(CXX) $(CXXFLAGS) -c -o $@ $^

clean:
	rm -f test_ds1603l ds1603l.o test_ds1603l.o

.PHONY: clean
