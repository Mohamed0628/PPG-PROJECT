# Host validation: run `make check` on any POSIX box with gcc.
CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -fsanitize=undefined \
          -I../components/ads131m02/include \
          -I../components/ppg_acq/include
LDLIBS  = -lm

FIR_SRC = ../components/ppg_acq/fir.c
ADC_SRC = ../components/ads131m02/ads131m02_frame.c \
          ../components/ppg_acq/ppg_stream.c

all: fir_test adc_frame_test

fir_test: fir_test.c $(FIR_SRC)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

adc_frame_test: adc_frame_test.c $(ADC_SRC) $(FIR_SRC)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

check: all
	./fir_test
	./adc_frame_test
	python3 ../tools/sinc_fir_check.py

clean:
	rm -f fir_test adc_frame_test
