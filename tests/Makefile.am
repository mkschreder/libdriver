AUTOTEST = $(AUTOM4TE) --language=autotest

TESTS =\
	m25
check_PROGRAMS=$(TESTS)

AM_CFLAGS=\
	$(COVERAGE_FLAGS)\
	-I$(abs_top_builddir)/include\
	-I$(srcdir)/../include
AM_CXXFLAGS=\
	$(COVERAGE_FLAGS)\
	-include gtest/gtest.h -include gmock/gmock.h\
	-ffunction-sections\
	-I$(abs_top_builddir)/include\
	-I$(srcdir)/../include\
	-L$(abs_top_builddir)/lib\
	-L/usr/local/lib
AM_LDFLAGS=\
	-Wl,--gc-sections -lm -lgtest -lgmock -lpthread -lfdt -lfirmware

mpu6050_SOURCES=\
	../src/accgyro/mpu6050.c\
	mpu6050.cpp

m25_SOURCES=\
	../src/mem/m25.c\
	m25.cpp

AUTOMAKE_OPTIONS=subdir-objects
